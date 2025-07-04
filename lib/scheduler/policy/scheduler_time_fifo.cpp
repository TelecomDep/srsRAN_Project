#include "scheduler_time_fifo.h"
#include "../slicing/slice_ue_repository.h"

using namespace srsran;

scheduler_time_fifo::scheduler_time_fifo(const scheduler_ue_expert_config& expert_cfg_) : expert_cfg(expert_cfg_) {}

void scheduler_time_fifo::compute_ue_dl_priorities(slot_point               pdcch_slot,
                                                    slot_point               pdsch_slot,
                                                    span<ue_newtx_candidate> ue_candidates)
    {

  int ue_index = 1;

  for (ue_newtx_candidate& candidate : ue_candidates) {
    candidate.priority = 1 / ue_index++;
  }
}

void scheduler_time_fifo::compute_ue_ul_priorities(slot_point               pdcch_slot,
                                                 slot_point               pusch_slot,
                                                 span<ue_newtx_candidate> ue_candidates)
{
  // \ref compute_ue_dl_priorities
   int current_priority = 1;

  for (ue_newtx_candidate& candidate : ue_candidates) {
    candidate.priority = 1/current_priority++;
  }
}

void scheduler_time_fifo::save_dl_newtx_grants(span<const dl_msg_alloc> dl_grants)
{
  if (dl_grants.empty()) {
    return;
  }

  // Mark the count for the allocated UEs.
  for (const auto& grant : dl_grants) {
    ue_last_dl_alloc_count[grant.context.ue_index] = dl_alloc_count;
  }
  ++dl_alloc_count;
}

void scheduler_time_fifo::save_ul_newtx_grants(span<const ul_sched_info> ul_grants)
{
  if (ul_grants.empty()) {
    return;
  }

  // Mark the count for the allocated UEs.
  for (const auto& grant : ul_grants) {
    ue_last_ul_alloc_count[grant.context.ue_index] = ul_alloc_count;
  }
  ++ul_alloc_count;
}
