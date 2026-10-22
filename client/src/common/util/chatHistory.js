/**
 * The history view's list: top-level chats only (a subagent belongs to its parent), filtered by
 * what the operator typed, newest first, in Today / Yesterday / Earlier groups.
 */

const startOfDay = (date) =>
  new Date(date.getFullYear(), date.getMonth(), date.getDate()).getTime();

/** @returns {'Today' | 'Yesterday' | 'Earlier'} */
export const dayGroupOf = (timestamp, now = new Date()) => {
  const when = new Date(timestamp);
  if (Number.isNaN(when.getTime())) return 'Earlier';
  const days = Math.round((startOfDay(now) - startOfDay(when)) / 86_400_000);
  if (days <= 0) return 'Today';
  if (days === 1) return 'Yesterday';
  return 'Earlier';
};

const matches = (chat, query) => {
  const needle = query.trim().toLowerCase();
  if (!needle) return true;
  return [chat.name, chat.preview, chat.agent].some((field) =>
    field?.toLowerCase().includes(needle),
  );
};

/**
 * @param {Array<object>} chats - `availableChats`
 * @param {string} query
 * @param {Date} [now]
 * @returns {Array<{label: string, chats: Array<object>}>} only the groups that have chats
 */
export const groupHistory = (chats, query = '', now = new Date()) => {
  const groups = { Today: [], Yesterday: [], Earlier: [] };

  chats
    .filter((chat) => !chat.parentChatId && matches(chat, query))
    .sort((a, b) => new Date(b.lastUpdated).getTime() - new Date(a.lastUpdated).getTime())
    .forEach((chat) => groups[dayGroupOf(chat.lastUpdated, now)].push(chat));

  return Object.entries(groups)
    .filter(([, list]) => list.length > 0)
    .map(([label, list]) => ({ label, chats: list }));
};

/** "10:41" for today's chats, "Mon 14" otherwise. */
export const historyTimeLabel = (timestamp, now = new Date()) => {
  const when = new Date(timestamp);
  if (Number.isNaN(when.getTime())) return '';
  if (dayGroupOf(timestamp, now) === 'Today') {
    return when.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
  }
  return when.toLocaleDateString([], { weekday: 'short', day: 'numeric' });
};
