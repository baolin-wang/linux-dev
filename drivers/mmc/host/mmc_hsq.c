// SPDX-License-Identifier: GPL-2.0
/*
 * MMC software queue support based on command queue interfaces
 *
 * Copyright (C) 2019 Linaro, Inc.
 * Author: Baolin Wang <baolin.wang@linaro.org>
 */

#include <linux/mmc/card.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>

#include "mmc_hsq.h"

#define HSQ_NUM_SLOTS	32
#define HSQ_INVALID_TAG	HSQ_NUM_SLOTS

#define HSQ_PACKED_FLUSH_BLOCKS	256
#define HSQ_PACKED_QUEUE_DEPTH	64

/**
 * mmc_hsq_packed_algo_rw - the algorithm to package read or write requests
 * @mmc: the host controller
 *
 * TODO: we can add more condition to decide if we can package this
 * request or not.
 */
void mmc_hsq_packed_algo_rw(struct mmc_host *mmc)
{
	struct mmc_hsq *hsq = mmc->cqe_private;
	struct hsq_packed *packed = hsq->packed;
	struct mmc_packed_request *prq = &packed->prq;
	struct mmc_request *mrq, *t;
	u32 i = 0;

	list_for_each_entry_safe(mrq, t, &packed->list, list) {
		if (++i > packed->max_entries)
			break;

		list_move_tail(&mrq->list, &prq->list);
		prq->nr_reqs++;
	}
}
EXPORT_SYMBOL_GPL(mmc_hsq_packed_algo_rw);

static void mmc_hsq_pump_requests(struct mmc_hsq *hsq)
{
	struct hsq_packed *packed = hsq->packed;
	struct mmc_host *mmc = hsq->mmc;
	struct hsq_slot *slot;
	struct mmc_request *mrq;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&hsq->lock, flags);

	/* Make sure we are not already running a request now */
	if (hsq->mrq || (packed && packed->prq.nr_reqs)) {
		spin_unlock_irqrestore(&hsq->lock, flags);
		return;
	}

	/* Make sure there are remain requests need to pump */
	if (!hsq->qcnt || !hsq->enabled) {
		spin_unlock_irqrestore(&hsq->lock, flags);
		return;
	}

	if (packed) {
		/* Try to package requests */
		packed->ops->packed_algo(mmc);

		packed->busy = true;
		hsq->qcnt -= packed->prq.nr_reqs;
	} else {
		slot = &hsq->slot[hsq->next_tag];
		hsq->mrq = slot->mrq;
		hsq->qcnt--;
	}

	spin_unlock_irqrestore(&hsq->lock, flags);

	if (!packed) {
		if (mmc->ops->request_atomic)
			mmc->ops->request_atomic(mmc, hsq->mrq);
		else
			mmc->ops->request(mmc, hsq->mrq);

		return;
	}

	if (packed->ops->prepare_hardware) {
		ret = packed->ops->prepare_hardware(mmc);
		if (ret) {
			pr_err("failed to prepare hardware\n");
			goto error;
		}
	}

	ret = packed->ops->packed_request(mmc, &packed->prq);
	if (ret) {
		pr_err("failed to packed requests\n");
		goto error;
	}

	return;

error:
	spin_lock_irqsave(&hsq->lock, flags);

	list_for_each_entry(mrq, &packed->prq.list, list) {
		struct mmc_data *data = mrq->data;

		data->error = ret;
		data->bytes_xfered = 0;
	}

	spin_unlock_irqrestore(&hsq->lock, flags);

	mmc_hsq_finalize_packed_request(mmc, &packed->prq);
}

static void mmc_hsq_update_next_tag(struct mmc_hsq *hsq, int remains)
{
	struct hsq_slot *slot;
	int tag;

	/*
	 * If there are no remain requests in software queue, then set a invalid
	 * tag.
	 */
	if (!remains) {
		hsq->next_tag = HSQ_INVALID_TAG;
		return;
	}

	/*
	 * Increasing the next tag and check if the corresponding request is
	 * available, if yes, then we found a candidate request.
	 */
	if (++hsq->next_tag != HSQ_INVALID_TAG) {
		slot = &hsq->slot[hsq->next_tag];
		if (slot->mrq)
			return;
	}

	/* Othersie we should iterate all slots to find a available tag. */
	for (tag = 0; tag < HSQ_NUM_SLOTS; tag++) {
		slot = &hsq->slot[tag];
		if (slot->mrq)
			break;
	}

	if (tag == HSQ_NUM_SLOTS)
		tag = HSQ_INVALID_TAG;

	hsq->next_tag = tag;
}

static void mmc_hsq_post_request(struct mmc_hsq *hsq)
{
	struct hsq_packed *packed = hsq->packed;
	bool need_pump;
	unsigned long flags;
	int remains;

	spin_lock_irqsave(&hsq->lock, flags);

	remains = hsq->qcnt;
	if (packed) {
		need_pump = !packed->rqs_pending;
		packed->prq.nr_reqs = 0;
	} else {
		need_pump = true;
		hsq->mrq = NULL;

		/* Update the next available tag to be queued. */
		mmc_hsq_update_next_tag(hsq, remains);
	}

	if (hsq->waiting_for_idle && !remains) {
		hsq->waiting_for_idle = false;
		wake_up(&hsq->wait_queue);
	}

	/* Do not pump new request in recovery mode. */
	if (hsq->recovery_halt) {
		spin_unlock_irqrestore(&hsq->lock, flags);
		return;
	}

	spin_unlock_irqrestore(&hsq->lock, flags);

	 /*
	 * For non-packed request mode, we should try to pump new request to
	 * host controller as fast as possible if there are pending requests,
	 * after completing previous request.
	 *
	 * For packed request mode, If there are not enough requests in queue
	 * and the request pending flag was set, then do not pump requests here,
	 * instead we should let the mmc_blk_swq_issue_rw_rq() combine more
	 * requests and pump them.
	 */
	if ((need_pump && remains > 0) ||
	    (packed && remains >= packed->max_entries))
		mmc_hsq_pump_requests(hsq);
}

/**
 * mmc_hsq_finalize_request - finalize one request if the request is done
 * @mmc: the host controller
 * @mrq: the request need to be finalized
 *
 * Return true if we finalized the corresponding request in software queue,
 * otherwise return false.
 */
bool mmc_hsq_finalize_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct mmc_hsq *hsq = mmc->cqe_private;
	unsigned long flags;

	spin_lock_irqsave(&hsq->lock, flags);

	if (!hsq->enabled || !hsq->mrq || hsq->mrq != mrq) {
		spin_unlock_irqrestore(&hsq->lock, flags);
		return false;
	}

	/*
	 * Clear current completed slot request to make a room for new request.
	 */
	hsq->slot[hsq->next_tag].mrq = NULL;

	spin_unlock_irqrestore(&hsq->lock, flags);

	mmc_cqe_request_done(mmc, hsq->mrq);

	mmc_hsq_post_request(hsq);

	return true;
}
EXPORT_SYMBOL_GPL(mmc_hsq_finalize_request);

/**
 * mmc_hsq_finalize_packed_request - finalize one packed request
 * @mmc: the host controller
 * @prq: the packed request need to be finalized
 */
void mmc_hsq_finalize_packed_request(struct mmc_host *mmc,
				     struct mmc_packed_request *prq)
{
	struct mmc_hsq *hsq = mmc->cqe_private;
	struct hsq_packed *packed = hsq->packed;
	struct mmc_request *mrq, *t;
	LIST_HEAD(head);
	unsigned long flags;

	if (!packed || !prq)
		return;

	if (packed->ops->unprepare_hardware &&
	    packed->ops->unprepare_hardware(mmc))
		pr_err("failed to unprepare hardware\n");

	/*
	 * Clear busy flag to allow collecting more requests into command
	 * queue, but now we can not pump them to controller, we should wait
	 * for all requests are completed. During the period of completing
	 * requests, we should collect more requests from block layer as much
	 * as possible.
	 */
	spin_lock_irqsave(&hsq->lock, flags);
	list_splice_tail_init(&prq->list, &head);
	packed->busy = false;
	spin_unlock_irqrestore(&hsq->lock, flags);

	list_for_each_entry_safe(mrq, t, &head, list) {
		list_del(&mrq->list);

		mmc_cqe_request_done(mmc, mrq);
	}

	mmc_hsq_post_request(hsq);
}
EXPORT_SYMBOL_GPL(mmc_hsq_finalize_packed_request);

static void mmc_hsq_commit_rqs(struct mmc_host *mmc, bool last)
{
	struct mmc_hsq *hsq = mmc->cqe_private;
	struct hsq_packed *packed = hsq->packed;

	if (!packed)
		return;

	spin_lock_irq(&hsq->lock);

	/* Set pending flag which indicates more request will be coming */
	if (!last && !packed->rqs_pending)
		packed->rqs_pending = true;

	spin_unlock_irq(&hsq->lock);

	/*
	 * If the last request is coming in hardware queue, then pump requests
	 * to controller as fast as possible.
	 */
	if (last)
		mmc_hsq_pump_requests(hsq);
}

static bool mmc_hsq_is_busy(struct mmc_host *mmc)
{
	struct mmc_hsq *hsq = mmc->cqe_private;
	struct hsq_packed *packed = hsq->packed;
	unsigned long flags;
	bool busy;

	spin_lock_irqsave(&hsq->lock, flags);

	/*
	 * For packed mode, when hardware is busy, we can only allow maximum
	 * packed number requests to be ready in software queue to be queued
	 * after previous packed request is completed, which avoiding long
	 * latency.
	 *
	 * For non-packed mode, we can only allow 2 requests in flight to avoid
	 * long latency.
	 *
	 * Otherwise return BLK_STS_RESOURCE to tell block layer to dispatch
	 * requests later.
	 */
	if (packed)
		busy = packed->busy && hsq->qcnt >= packed->max_entries;
	else
		busy = hsq->qcnt > 1;

	spin_unlock_irqrestore(&hsq->lock, flags);

	return busy;
}

static void mmc_hsq_recovery_start(struct mmc_host *mmc)
{
	struct mmc_hsq *hsq = mmc->cqe_private;
	unsigned long flags;

	spin_lock_irqsave(&hsq->lock, flags);

	hsq->recovery_halt = true;

	spin_unlock_irqrestore(&hsq->lock, flags);
}

static void mmc_hsq_recovery_finish(struct mmc_host *mmc)
{
	struct mmc_hsq *hsq = mmc->cqe_private;
	int remains;

	spin_lock_irq(&hsq->lock);

	hsq->recovery_halt = false;
	remains = hsq->qcnt;

	spin_unlock_irq(&hsq->lock);

	/*
	 * Try to pump new request if there are request pending in software
	 * queue after finishing recovery.
	 */
	if (remains > 0)
		mmc_hsq_pump_requests(hsq);
}

static int mmc_hsq_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct mmc_hsq *hsq = mmc->cqe_private;
	struct hsq_packed *packed = hsq->packed;
	int nr_rqs = 0, tag = mrq->tag;

	spin_lock_irq(&hsq->lock);

	if (!hsq->enabled) {
		spin_unlock_irq(&hsq->lock);
		return -ESHUTDOWN;
	}

	/* Do not queue any new requests in recovery mode. */
	if (hsq->recovery_halt) {
		spin_unlock_irq(&hsq->lock);
		return -EBUSY;
	}

	hsq->qcnt++;

	if (packed) {
		list_add_tail(&mrq->list, &packed->list);

		/* New request comes, then clear pending flag */
		if (packed->rqs_pending)
			packed->rqs_pending = false;

		nr_rqs = hsq->qcnt;
	} else {
		hsq->slot[tag].mrq = mrq;

		/*
		 * Set the next tag as current request tag if no available
		 * next tag.
		 */
		if (hsq->next_tag == HSQ_INVALID_TAG)
			hsq->next_tag = tag;
	}

	spin_unlock_irq(&hsq->lock);

	/*
	 * For non-packed request mode, we should pump requests as soon as
	 * possible.
	 *
	 * For the packed request mode, if it is a larger request or the
	 * request count is larger than the maximum packed number, we
	 * should pump requests to controller. Otherwise we should try to
	 * combine requests as much as we can.
	 */
	if (!packed || mrq->data->blocks > HSQ_PACKED_FLUSH_BLOCKS ||
	    nr_rqs >= packed->max_entries)
		mmc_hsq_pump_requests(hsq);

	return 0;
}

static void mmc_hsq_post_req(struct mmc_host *mmc, struct mmc_request *mrq)
{
	if (mmc->ops->post_req)
		mmc->ops->post_req(mmc, mrq, 0);
}

static bool mmc_hsq_queue_is_idle(struct mmc_hsq *hsq, int *ret)
{
	struct hsq_packed *packed = hsq->packed;
	bool is_idle;

	spin_lock_irq(&hsq->lock);

	if (packed)
		is_idle = (!packed->prq.nr_reqs && !hsq->qcnt) ||
			hsq->recovery_halt;
	else
		is_idle = (!hsq->mrq && !hsq->qcnt) ||
			hsq->recovery_halt;

	*ret = hsq->recovery_halt ? -EBUSY : 0;
	hsq->waiting_for_idle = !is_idle;

	spin_unlock_irq(&hsq->lock);

	return is_idle;
}

static int mmc_hsq_wait_for_idle(struct mmc_host *mmc)
{
	struct mmc_hsq *hsq = mmc->cqe_private;
	int ret;

	wait_event(hsq->wait_queue,
		   mmc_hsq_queue_is_idle(hsq, &ret));

	return ret;
}

static void mmc_hsq_disable(struct mmc_host *mmc)
{
	struct mmc_hsq *hsq = mmc->cqe_private;
	u32 timeout = 500;
	int ret;

	spin_lock_irq(&hsq->lock);

	if (!hsq->enabled) {
		spin_unlock_irq(&hsq->lock);
		return;
	}

	spin_unlock_irq(&hsq->lock);

	ret = wait_event_timeout(hsq->wait_queue,
				 mmc_hsq_queue_is_idle(hsq, &ret),
				 msecs_to_jiffies(timeout));
	if (ret == 0) {
		pr_warn("could not stop mmc software queue\n");
		return;
	}

	spin_lock_irq(&hsq->lock);

	hsq->enabled = false;

	spin_unlock_irq(&hsq->lock);
}

static int mmc_hsq_enable(struct mmc_host *mmc, struct mmc_card *card)
{
	struct mmc_hsq *hsq = mmc->cqe_private;

	spin_lock_irq(&hsq->lock);

	if (hsq->enabled) {
		spin_unlock_irq(&hsq->lock);
		return -EBUSY;
	}

	hsq->enabled = true;

	spin_unlock_irq(&hsq->lock);

	return 0;
}

static const struct mmc_cqe_ops mmc_hsq_ops = {
	.cqe_enable = mmc_hsq_enable,
	.cqe_disable = mmc_hsq_disable,
	.cqe_request = mmc_hsq_request,
	.cqe_post_req = mmc_hsq_post_req,
	.cqe_wait_for_idle = mmc_hsq_wait_for_idle,
	.cqe_recovery_start = mmc_hsq_recovery_start,
	.cqe_recovery_finish = mmc_hsq_recovery_finish,
	.cqe_is_busy = mmc_hsq_is_busy,
	.cqe_commit_rqs = mmc_hsq_commit_rqs,
};

int mmc_hsq_init(struct mmc_hsq *hsq, struct mmc_host *mmc,
		 const struct hsq_packed_ops *ops, int max_packed)
{
	if (ops && max_packed > 1) {
		struct hsq_packed *packed;

		packed = devm_kzalloc(mmc_dev(mmc), sizeof(struct hsq_packed),
				      GFP_KERNEL);
		if (!packed)
			return -ENOMEM;

		packed->ops = ops;
		packed->max_entries = max_packed;
		INIT_LIST_HEAD(&packed->list);
		INIT_LIST_HEAD(&packed->prq.list);

		hsq->packed = packed;
		mmc->cqe_qdepth = HSQ_PACKED_QUEUE_DEPTH;
		mmc->max_packed_reqs = max_packed;
	} else {
		hsq->num_slots = HSQ_NUM_SLOTS;
		hsq->next_tag = HSQ_INVALID_TAG;
		mmc->cqe_qdepth = HSQ_NUM_SLOTS;

		hsq->slot = devm_kcalloc(mmc_dev(mmc), hsq->num_slots,
					 sizeof(struct hsq_slot),
					 GFP_KERNEL);
		if (!hsq->slot)
			return -ENOMEM;
	}

	hsq->mmc = mmc;
	hsq->mmc->cqe_private = hsq;
	mmc->cqe_ops = &mmc_hsq_ops;

	spin_lock_init(&hsq->lock);
	init_waitqueue_head(&hsq->wait_queue);

	return 0;
}
EXPORT_SYMBOL_GPL(mmc_hsq_init);

void mmc_hsq_suspend(struct mmc_host *mmc)
{
	mmc_hsq_disable(mmc);
}
EXPORT_SYMBOL_GPL(mmc_hsq_suspend);

int mmc_hsq_resume(struct mmc_host *mmc)
{
	return mmc_hsq_enable(mmc, NULL);
}
EXPORT_SYMBOL_GPL(mmc_hsq_resume);
