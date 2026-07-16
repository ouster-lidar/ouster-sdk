(function () {
  const ICON = `
    <svg aria-hidden="true" viewBox="0 0 24 24" class="copy-link-svg">
      <path d="M10 14a3 3 0 0 0 3 3h3a3 3 0 0 0 0-6h-1"
            fill="none" stroke="currentColor" stroke-linecap="round"
            stroke-linejoin="round" stroke-width="1.6" />
      <path d="M14 10a3 3 0 0 0-3-3H8a3 3 0 0 0 0 6h1"
            fill="none" stroke="currentColor" stroke-linecap="round"
            stroke-linejoin="round" stroke-width="1.6" />
    </svg>`;

  function copyToClipboard(text) {
    if (navigator.clipboard && navigator.clipboard.writeText) {
      return navigator.clipboard.writeText(text);
    }
    return new Promise(function (resolve, reject) {
      const textarea = document.createElement('textarea');
      textarea.value = text;
      textarea.style.position = 'fixed';
      textarea.style.opacity = '0';
      document.body.appendChild(textarea);
      textarea.focus();
      textarea.select();
      try {
        const successful = document.execCommand('copy');
        document.body.removeChild(textarea);
        successful ? resolve() : reject();
      } catch (err) {
        document.body.removeChild(textarea);
        reject(err);
      }
    });
  }

  function wireHeaderLink(link) {
    if (link.dataset.copyHeadingLinked === 'true') {
      return;
    }
    const anchor = determineAnchor(link);

    link.classList.add('copy-link-icon');
    if (!link.querySelector('img') && !link.querySelector('svg')) {
      link.innerHTML = ICON;
    }
    link.setAttribute('title', 'Copy link to this section');
    link.setAttribute('aria-label', 'Copy link to this section');
    link.dataset.copyHeadingLinked = 'true';

    const copyHandler = function (event) {
      event.preventDefault();
      const target = anchor || '';
      const url = buildCopyUrl(target);
      copyToClipboard(url).then(function () {
        showCopiedToast(link);
        if (target) {
          window.history.replaceState(null, '', target);
        }
      }).catch(function () {
        // No-op; we silently fail to avoid disruptive alerts.
      });
    };

    link.addEventListener('click', copyHandler);
    link.addEventListener('keydown', function (event) {
      if (event.key === 'Enter' || event.key === ' ') {
        copyHandler(event);
      }
    });
  }

  function init() {
    const links = document.querySelectorAll('a.headerlink');
    links.forEach(wireHeaderLink);
  }

  function determineAnchor(link) {
    const rawHref = link.getAttribute('href') || '';
    if (rawHref.startsWith('#')) {
      return rawHref;
    }
    try {
      const parsed = new URL(rawHref, window.location.href);
      if (parsed.hash) {
        return parsed.hash;
      }
    } catch (err) {
      // ignore malformed URLs and fall back to nearest id
    }
    const owner = link.closest('[id]');
    if (owner && owner.id) {
      return `#${owner.id}`;
    }
    return '';
  }

  function buildCopyUrl(anchor) {
    const base = `${window.location.origin}${window.location.pathname}`;
    if (!anchor) {
      return base;
    }
    return `${base}${anchor}`;
  }

  function showCopiedToast(link) {
    const existing = document.querySelector('.copy-link-toast');
    if (existing) {
      existing.remove();
    }

    const toast = document.createElement('div');
    toast.className = 'copy-link-toast';
    toast.textContent = 'Link copied';

    const rect = link.getBoundingClientRect();
    const left = rect.left + rect.width / 2;
    const top = rect.top + window.scrollY - 12;

    toast.style.left = `${left}px`;
    toast.style.top = `${top}px`;

    document.body.appendChild(toast);

    setTimeout(function () {
      toast.remove();
    }, 2000);
  }

  const observer = new MutationObserver(init);

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', () => {
      init();
      observer.observe(document.body, { childList: true, subtree: true });
    });
  } else {
    init();
    observer.observe(document.body, { childList: true, subtree: true });
  }

  window.addEventListener('load', init);
})();
