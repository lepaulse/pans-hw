# SPDX-License-Identifier: Apache-2.0
# The cell groups of a routed stream unit below u_dut/, for the stage report and the power by register group.
# Names are as yosys leaves them. A flop is named after its register, or after the output port it drives.
# A register no pattern names falls into other_flops or control, so a rename moves a number and never loses it.
#
#   stream_power_group <unit> <name> <master>   power group of one unit cell (name without u_dut/)
#   stream_power_groups <unit>                  those groups, in report order
#   stream_stage_group <unit> <name> <master>   stage group of one sequential unit cell ("" for the others);
#                                               the encoder's qd_* give "pipe", which stream_stages.tcl splits by path
#   stream_stage_groups <unit>                  the stage groups, in pipeline order
#   stream_seq <master>                         1 iff the master is a flop, a latch or a clock gate (Nangate45:
#                                               DFF, SDFF, DLH, DLL, CLKGATE; ASAP7: DFF, DHL, DLL, ICG)

proc stream_seq {master} {
  foreach p {DFF* SDFF* DLH* DLL* CLKGATE* DHL* ICG*} { if {[string match $p $master]} { return 1 } }
  return 0
}

proc stream_name {name} {
  regsub -all {\\} $name {} n
  return $n
}

# ---- Power groups -------------------------------------------------------------------------------------------------

proc stream_power_groups {unit} {
  if {$unit eq "dec"} {
    return {unit_comb core_comb_named clock_gates core_flops lane_state wf_mem of_mem scale_q tag_pipe counters control}
  }
  return {unit_comb core_comb clock_gates core_flops input_reg tag_pipe pipe_div pipe_f pipe_c pipe_lane pipe_valid
          lane_x lane_ctl fifo_mem fifo_ctl counters control other_flops}
}

proc stream_power_group {unit name master} {
  set n [stream_name $name]
  if {$unit eq "dec"} {
    # decoder: flops are DFF and SDFF masters, the gates' latches a group of their own
    if {[string match "*u_cg*" $n] && [stream_seq $master]} { return clock_gates }
    if {![string match "DFF*" $master] && ![string match "SDFF*" $master]} {
      if {[string match "u_core.*" $n]} { return core_comb_named }
      return unit_comb
    }
    foreach {g pats} {
      core_flops {u_core.*}
      lane_state {x_q* at_L_q* second_word_q* g_lane*}
      wf_mem     {wf_mem_q* g_word*}
      of_mem     {of_mem_q* g_out*}
      scale_q    {scale_q* g_scale*}
      tag_pipe   {q4_x* q5_* q?_lane* q6_* q?_valid*}
      counters   {sym_left_q* in_bytes_left_q* done_out_* sym_out_t* sym_out_lane* words_consumed* n_escapes* out_t_q*
                  out_lane_q*}
    } { foreach p $pats { if {[string match $p $n]} { return $g } } }
    return control
  }
  # encoder: flops are DFF masters, the gates' latches found by name
  if {[string match "*u_cg*" $n]} { return clock_gates }
  if {![string match "DFF*" $master]} {
    if {[string match "u_core.*" $n]} { return core_comb }
    return unit_comb
  }
  foreach {p g} {
    "qd_div*" pipe_div "qd_f*" pipe_f "qd_c*" pipe_c "qd_lane*" pipe_lane "qd_valid*" pipe_valid
    "lane_x*" lane_x "g_lane*" lane_x "fresh*" lane_ctl "busy*" lane_ctl
    "fifo_bank*" fifo_mem "fifo_*" fifo_ctl "cnt_escapes*" counters "g_esc*" counters
    "job_rem*" counters "done_out_n_*" counters "done_out_lane*" counters "done_out_err*" counters "exp_lane*" counters
    "u_core.*" core_flops "q0_*" input_reg "q1_*" tag_pipe "q2_*" tag_pipe "st*" control
  } { if {[string match $p $n]} { return $g } }
  return other_flops
}

# ---- Stage groups: where a register-to-register path ends -----------------------------------------------------------

proc stream_stage_groups {unit} {
  if {$unit eq "dec"} {
    return {issue_core_s1 core_s2 core_s3 core_s4 q4_x mul_q5 renorm_x renorm_flags renorm_wf renorm_of tag_pipe
            issue_ctrl scale counters control cg_enable cg_check}
  }
  return {input core_s1 core_s2 tag_pipe renorm divider lane_write fifo_mem fifo_ctl lane_flags counters control
          cg_enable cg_check}
}

proc stream_stage_group {unit name master} {
  if {![stream_seq $master]} { return "" }
  set n [stream_name $name]
  if {[string match "*u_cg*" $n]} { return cg_enable }
  if {$unit eq "dec"} {
    foreach {g pats} {
      issue_core_s1 {u_core.q1_*}
      core_s2       {u_core.q2_*}
      core_s3       {u_core.q3_*}
      core_s4       {u_core.out_*}
      q4_x          {q4_x*}
      mul_q5        {q5_*}
      renorm_x      {x_q* g_lane*.x*}
      renorm_flags  {at_L_q* second_word_q* g_lane*.at_L* g_lane*.second_word*}
      renorm_wf     {wf_* g_word*}
      renorm_of     {of_* g_out*}
      issue_ctrl    {rr_lane_q* q1_* reserved_q* credit_q* lane_busy_q*}
      tag_pipe      {q2_* q3_* q4_* q6_*}
      scale         {scale_q* g_scale* scale_present_q* scale_more_q*}
      counters      {sym_left_q* in_bytes_left_q* *words_consumed* *n_escapes* sym_out_* out_t_q* out_lane_q*}
    } { foreach p $pats { if {[string match $p $n]} { return $g } } }
    return control
  }
  if {[string match "qd_*" $n]} { return pipe }
  foreach {g pats} {
    input      {q0_*}
    core_s1    {u_core.q1_*}
    core_s2    {u_core.*}
    tag_pipe   {q1_* q2_*}
    lane_write {lane_x* g_lane*}
    fifo_mem   {fifo_bank*}
    fifo_ctl   {fifo_*}
    lane_flags {fresh* busy*}
    counters   {job_rem* cnt_* g_esc* done_out_* exp_lane*}
  } { foreach p $pats { if {[string match $p $n]} { return $g } } }
  return control
}
