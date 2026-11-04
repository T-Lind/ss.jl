export const APP_VERSION = '0.9.2';

function stamp() {
  for (const el of document.querySelectorAll('[data-appver]'))
    el.textContent = 'v' + APP_VERSION;
  document.documentElement.dataset.appver = APP_VERSION;
  const links = document.querySelector('.appnav-links');
  const current = links?.querySelector('[aria-current="page"]');
  if (current && links.scrollWidth > links.clientWidth) {
    const delta = current.getBoundingClientRect().right - links.getBoundingClientRect().right;
    if (delta > 0) links.scrollLeft += delta;
  }
}

if (typeof document !== 'undefined') {
  if (document.readyState === 'loading')
    document.addEventListener('DOMContentLoaded', stamp, { once: true });
  else
    stamp();
}
