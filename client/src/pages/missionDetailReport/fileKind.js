// Single source of truth for classifying mission files by kind in the report.
// DJI files arrive UPPERCASE (`.JPG`, `.MP4`), so every check is case-insensitive
// — the previous `.endsWith('.jpg')` matched nothing and hid every image.

const IMAGE_EXTS = ['.jpg', '.jpeg', '.png', '.tif', '.tiff', '.webp', '.gif'];
const VIDEO_EXTS = ['.mp4', '.mov', '.avi', '.mkv', '.webm'];

const extOf = (name) => {
  if (!name) return '';
  const dot = name.lastIndexOf('.');
  return dot === -1 ? '' : name.slice(dot).toLowerCase();
};

export const isImageFile = (name) => IMAGE_EXTS.includes(extOf(name));
export const isVideoFile = (name) => VIDEO_EXTS.includes(extOf(name));

// 'image' | 'video' | 'other' — keeps callers from repeating the ext lists.
export const fileKind = (name) => {
  if (isImageFile(name)) return 'image';
  if (isVideoFile(name)) return 'video';
  return 'other';
};
