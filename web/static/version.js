export const APP_VERSION = '0.6.0';

function stamp() {
  for (const el of document.querySelectorAll('[data-appver]'))
    el.textContent = 'v' + APP_VERSION;
  document.documentElement.dataset.appver = APP_VERSION;
}

if (typeof document !== 'undefined') {
  if (document.readyState === 'loading')
    document.addEventListener('DOMContentLoaded', stamp, { once: true });
  else
    stamp();
}
