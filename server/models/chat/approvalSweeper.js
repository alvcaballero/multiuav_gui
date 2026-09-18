import { MessageOrchestrator } from './chat.js';
import { chatLogger } from '../../common/logger.js';
import { TOOL_APPROVAL_ENFORCE, TOOL_APPROVAL_SWEEP_INTERVAL_MS } from '../../config/config.js';

/**
 * Ages out tool approvals nobody answered and unparks the turns waiting on them.
 *
 * Without this, an unanswered approval parks its turn forever, leaving a
 * `function_call` in history that no `function_call_output` ever answers — which
 * the providers reject on the NEXT request, so the chat breaks long after the
 * approval everyone forgot about.
 */
class ApprovalSweeper {
  constructor() {
    this.timer = null;
    this.running = false;
  }

  async sweep() {
    // Overlapping sweeps would resume the same turn twice; a tick that lands on a
    // slow one is simply skipped, since the next is 30s away.
    if (this.running) return;
    this.running = true;
    try {
      await MessageOrchestrator.sweepExpiredApprovals();
    } catch (error) {
      chatLogger.error('[ToolApproval] Sweep failed:', error);
    } finally {
      this.running = false;
    }
  }

  start() {
    if (this.timer || !TOOL_APPROVAL_ENFORCE) return;
    this.timer = setInterval(() => this.sweep(), TOOL_APPROVAL_SWEEP_INTERVAL_MS);
    this.timer.unref?.();
    chatLogger.info(`[ToolApproval] Sweeper started (every ${TOOL_APPROVAL_SWEEP_INTERVAL_MS}ms)`);
  }

  stop() {
    if (!this.timer) return;
    clearInterval(this.timer);
    this.timer = null;
  }
}

export const approvalSweeper = new ApprovalSweeper();
