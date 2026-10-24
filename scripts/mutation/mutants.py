# Mutants of core/src/candidates/cand_obj/cand_obj.cpp for scripts/mutation/run_mutants.sh (final review S6).
# Three are EQUIVALENT (the mutated check is redundant with another one) and are expected to survive:
#   seedInsideSprayed_removed  - a crown whose top is inside a sprayed tree is matched to it by findInstanceForCrown
#                                (same hull / match_radius test) and dropped as crowns_dropped_sprayed_tree first;
#   heightRefresh_inLocked_only - no cell height is fused in a locked phase, so refreshing the quantile is a no-op;
#   confirm_inLockedPhase      - unconfirmed instances are not updatable in a locked phase, so the confirm branch is
#                                never reached there.
M = [
 ("insideSprayed_false", [("bool CandObj::insideSprayed(const Point2& p) const {\n", "bool CandObj::insideSprayed(const Point2& p) const {\n  if (true) return false;\n")]),
 ("seedInsideSprayed_removed", [('      if (insideSprayed(c.peak)) { count("crowns_dropped_inside_sprayed"); continue; }\n', "")]),
 ("confirmInsideSprayed_removed", [('      if (insideSprayed(in.centre)) { count("instances_refused_confirm_inside_sprayed"); continue; }\n', "")]),
 ("mergeSprayed_allowed", [("        if (ea.sprayed || eb.sprayed) continue;\n", ""),
                           ("      if (in.sprayed || in.n_latched == 0) continue;\n      refreshHull(in);", "      if (in.n_latched == 0) continue;\n      refreshHull(in);"),
                           ("      if (ea.sprayed || ea.n_latched == 0) continue;", "      if (ea.n_latched == 0) continue;")]),
 ("mergeWinner_tentativeWins", [("const bool a_keeps = ea.confirmed != eb.confirmed ? ea.confirmed", "const bool a_keeps = ea.confirmed != eb.confirmed ? eb.confirmed")]),
 ("mergeWinner_noConfirmedRung", [("const bool a_keeps = ea.confirmed != eb.confirmed ? ea.confirmed", "const bool a_keeps = false ? ea.confirmed")]),
 ("heightFused_inLocked_zcap", [("pol.may_create ? z_cap : -std::numeric_limits<double>::infinity()", "z_cap")]),
 ("heightFused_inLocked_both", [("pol.may_create ? z_cap : -std::numeric_limits<double>::infinity()", "z_cap"), ("      if (pol.may_create) refreshHeight(in);\n", "      refreshHeight(in);\n")]),
 ("heightRefresh_inLocked_only", [("      if (pol.may_create) refreshHeight(in);\n", "      refreshHeight(in);\n")]),
 ("markSprayed_unpublished", [("    if (!publishable(in)) break;\n", "")]),
 ("freeze_nonTarget_removed", [("    if (pol.freeze_non_target && in.id != target) continue;\n", "")]),
 ("lockedUnconfirmed_grow", [("    if (!pol.may_create && !in.confirmed) continue;", "    if (false) continue;")]),
 ("confirm_inLockedPhase", [("if (!in.confirmed && pol.may_create && in.frames_seen", "if (!in.confirmed && in.frames_seen")]),
 ("postSprayHold_noFreeze", [("      pol.freeze_non_target = true;\n      pol.may_merge = false;\n", "      pol.may_merge = false;\n")]),
 ("rateLimit_height_removed", [("    in.height_pub += std::clamp(d, -lim, lim);", "    in.height_pub += d;")]),
 ("stickyLock_never", [("    list_locked_ = true;\n", "    list_locked_ = false;\n")]),
 ("truncatedGuard_removed", [('      if (in.n_good_height_cells < cfg_.min_height_cells) { count("instances_waiting_untruncated_height"); stepPublished(in, t); continue; }\n', "")]),
 ("nanConfig_accepted", [('  if (!std::isfinite(static_cast<double>(v))) {\n', '  if (false) {\n')]),
 ("newTree_inLockedPhase", [('      if (!pol.may_create) { count("crowns_dropped_new_tree_locked_phase"); continue; }\n', "")]),
 ("merge_inLockedPhase", [("  if (pol.may_merge) mergeInstances(t);", "  mergeInstances(t);")]),
]
