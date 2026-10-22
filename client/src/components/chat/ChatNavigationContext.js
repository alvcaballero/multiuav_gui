import { createContext } from 'react';

/**
 * How a row of the transcript opens another chat (a subagent's card has an "Open chat" button).
 * Provided by the panel and read through context, not passed down as a prop: a function prop that is
 * new on every render would defeat the `memo` that keeps the transcript from re-rendering whole.
 */
export const ChatNavigationContext = createContext({ openChat: null });
