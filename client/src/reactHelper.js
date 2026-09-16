import { useRef, useEffect, useCallback } from 'react';
import { useDispatch } from 'react-redux';
import { errorsActions } from './store';

export const usePrevious = (value) => {
  const ref = useRef();
  useEffect(() => {
    ref.current = value;
  });
  return ref.current;
};

export const useAsyncTask = (effect, deps) => {
  const dispatch = useDispatch();
  useEffect(() => {
    const controller = new AbortController();
    let cleanup;
    effect({ signal: controller.signal })
      .then((result) => {
        cleanup = result;
      })
      .catch((error) => {
        if (error.name !== 'AbortError') {
          dispatch(errorsActions.push(error.message));
        }
      });
    return () => {
      controller.abort();
      cleanup?.();
    };
    // `effect` is intentionally excluded (caller passes inline functions), `deps` is caller-provided
    // eslint-disable-next-line @eslint-react/exhaustive-deps
  }, [...deps, dispatch]); // oxlint-disable-line react-doctor/exhaustive-deps
};

export const useCatch = (method) => {
  const dispatch = useDispatch();
  return (...parameters) => method(...parameters).catch((error) => dispatch(errorsActions.push(error.message)));
};

export const useCatchCallback = (method, deps) => {
  const dispatch = useDispatch();
  return useCallback(
    (...parameters) =>
      method(...parameters).catch((error) => dispatch(errorsActions.push(error.message))),
    // `deps` is caller-provided, same pattern as useAsyncTask
    // eslint-disable-next-line @eslint-react/exhaustive-deps
    [...deps, dispatch], // oxlint-disable-line react-doctor/exhaustive-deps
  );
};
