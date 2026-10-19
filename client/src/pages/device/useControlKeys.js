import { useEffect, useRef } from 'react';
import { KEY_MAP } from './manualControl';

const isTyping = (target) =>
  /INPUT|TEXTAREA|SELECT/.test(target.tagName) || Boolean(target.isContentEditable);

/**
 * Keyboard for the manual control: W/S A/D R/F Q/E move, I/K J/L gimbal, Enter applies,
 * Esc clears. Ignored while `enabled` is false or focus is in a form field.
 * The window listener is registered once; it reads the latest options from a ref that is
 * updated after each commit (never during render).
 */
export const useControlKeys = ({ enabled, gimbal, onNudge, onApply, onClear }) => {
  const optionsRef = useRef(null);

  useEffect(() => {
    optionsRef.current = { enabled, gimbal, onNudge, onApply, onClear };
  });

  useEffect(() => {
    const listener = (event) => {
      const options = optionsRef.current;
      if (!options?.enabled || isTyping(event.target)) return;
      const key = event.key.length === 1 ? event.key.toLowerCase() : event.key;
      if (KEY_MAP[key]) {
        const [field, sign] = KEY_MAP[key];
        if ((field === 'gp' || field === 'gy') && !options.gimbal) return;
        event.preventDefault();
        options.onNudge({ [field]: sign });
      } else if (key === 'Enter') {
        event.preventDefault();
        options.onApply();
      } else if (key === 'Escape') {
        options.onClear();
      }
    };
    window.addEventListener('keydown', listener);
    return () => window.removeEventListener('keydown', listener);
  }, []);
};
