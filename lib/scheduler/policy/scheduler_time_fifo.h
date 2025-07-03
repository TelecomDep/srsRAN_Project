#pragma once

#include "scheduler_policy.h"
#include "srsran/scheduler/config/scheduler_expert_config.h"

namespace srsran {

class scheduler_time_fifo : public scheduler_policy
{
public:
  scheduler_time_fifo(const scheduler_ue_expert_config& expert_cfg_);

  void add_ue(du_ue_index_t ue_index) override {}

  void rem_ue(du_ue_index_t ue_index) override {}

  void compute_ue_dl_priorities(slot_point               pdcch_slot,
                                slot_point               pdsch_slot,
                                span<ue_newtx_candidate> ue_candidates) override;

  void compute_ue_ul_priorities(slot_point               pdcch_slot,
                                slot_point               pusch_slot,
                                span<ue_newtx_candidate> ue_candidates) override;

  void save_dl_newtx_grants(span<const dl_msg_alloc> dl_grants) override;

  void save_ul_newtx_grants(span<const ul_sched_info> ul_grants) override;

private:
  const scheduler_ue_expert_config expert_cfg;

  // Tables to keep track of UE priorities.
  std::array<unsigned, MAX_NOF_DU_UES> ue_last_dl_alloc_count{};
  std::array<unsigned, MAX_NOF_DU_UES> ue_last_ul_alloc_count{};

  unsigned dl_alloc_count{0};
  unsigned ul_alloc_count{0};
};

} // namespace srsran