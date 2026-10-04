// Shared behavior for the compact controls used by both front ends.
export function escapeHTML(value) {
  return String(value ?? '').replace(/[&<>"']/g, c =>
    ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' })[c]);
}

export function allowsShortcut(event) {
  if (event.defaultPrevented || event.isComposing || event.ctrlKey || event.metaKey || event.altKey)
    return false;
  const target = event.target;
  if (target?.closest?.('input, select, textarea, [contenteditable]:not([contenteditable="false"]), dialog[open], [role="dialog"][aria-modal="true"]'))
    return false;
  // Space and Enter belong to the focused action's native activation.
  if ([' ', 'Enter'].includes(event.key) && target?.closest?.('button, a, [role="button"]'))
    return false;
  return true;
}

/** Give a compact span action the same keyboard activation as a button. */
const keyboardActions = new WeakSet();
export function keyboardAction(element) {
  if (element.tagName === 'BUTTON' || element.tagName === 'A' || keyboardActions.has(element)) return element;
  keyboardActions.add(element);
  element.setAttribute('role', 'button');
  element.tabIndex = 0;
  element.addEventListener('keydown', event => {
    if (!['Enter', ' '].includes(event.key) || event.ctrlKey || event.metaKey || event.altKey) return;
    event.preventDefault();
    event.stopPropagation();
    if (!event.repeat) element.click();
  });
  return element;
}
