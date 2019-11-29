// SPDX-License-Identifier: GPL-2.0
#ifndef LINUX_MMC_HSQ_H
#define LINUX_MMC_HSQ_H

struct hsq_packed_ops {
	void (*packed_algo)(struct mmc_host *mmc);
	int (*prepare_hardware)(struct mmc_host *mmc);
	int (*unprepare_hardware)(struct mmc_host *mmc);
	int (*packed_request)(struct mmc_host *mmc,
			      struct mmc_packed_request *prq);
};

struct hsq_packed {
	bool busy;
	bool rqs_pending;
	int max_entries;

	struct list_head list;
	struct mmc_packed_request prq;
	const struct hsq_packed_ops *ops;
};

struct hsq_slot {
	struct mmc_request *mrq;
};

struct mmc_hsq {
	struct mmc_host *mmc;
	struct mmc_request *mrq;
	wait_queue_head_t wait_queue;
	struct hsq_slot *slot;
	spinlock_t lock;

	int next_tag;
	int num_slots;
	int qcnt;

	bool enabled;
	bool waiting_for_idle;
	bool recovery_halt;

	struct hsq_packed *packed;
};

int mmc_hsq_init(struct mmc_hsq *hsq, struct mmc_host *mmc,
		 const struct hsq_packed_ops *ops, int max_packed);
void mmc_hsq_suspend(struct mmc_host *mmc);
int mmc_hsq_resume(struct mmc_host *mmc);
bool mmc_hsq_finalize_request(struct mmc_host *mmc, struct mmc_request *mrq);
void mmc_hsq_finalize_packed_request(struct mmc_host *mmc,
				     struct mmc_packed_request *prq);
void mmc_hsq_packed_algo_rw(struct mmc_host *mmc);

#endif
