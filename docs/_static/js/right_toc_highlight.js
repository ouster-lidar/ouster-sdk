(function () {
  /**
   * Scroll-spy and click handling for the Right TOC sidebar.
   * for Ouster SDK documentation theme.
   */
  function initRightToc() {
    const toc = document.querySelector('.right-toc');
    const scroller = document.querySelector('.content-with-rail');
    if (!toc || !scroller) return;

    const links = Array.from(toc.querySelectorAll('a[href]'));
    const headings = links.map(link => {
      const href = link.getAttribute('href');
      const id = link.dataset.targetId || extractIdFromHref(href);
      const isPageTop = (href === '#' || id === '');
      const el = isPageTop ? (document.querySelector('h1') || document.body) : document.getElementById(id);
      return { link, el, id: id || 'page-top', isPageTop };
    }).filter(h => h.el);

    if (!headings.length) return;
    const lastHeading = headings[headings.length - 1];
    let currentId = null;

    const setActive = id => {
      if (!id || currentId === id) return;
      currentId = id;
      headings.forEach(item => {
        const isActive = item.id === id;
        item.link.classList.toggle('active', isActive);
        if (isActive) ensureLinkVisible(item.link, toc);
      });
    };

    let updateScheduled = false;

    // Determines active section based on scroll progress.
    const update = () => {
      updateScheduled = false;
      const vh = window.innerHeight;
      const scrollY = scroller.scrollTop;
      const triggerTop = Math.floor(vh * 0.25);
      const isAtBottom = (scrollY + scroller.clientHeight) >= (scroller.scrollHeight - 25);

      let candidateId = null;
      for (let i = headings.length - 1; i >= 0; i--) {
        const h = headings[i];
        if (h.isPageTop) continue;
        if (Math.round(h.el.getBoundingClientRect().top) <= triggerTop) {
          candidateId = h.id;
          break;
        }
      }

      const finalId = candidateId || 
                     (isAtBottom && scrollY > 200 ? lastHeading.id : 
                     (scrollY > 100 && headings[1] ? headings[1].id : headings[0].id));

      setActive(finalId);

      // When content reaches page bottom, scroll the right-toc list to its bottom
      if (isAtBottom && toc.scrollHeight > toc.clientHeight) {
        toc.scrollTop = toc.scrollHeight - toc.clientHeight;
      }
    };

    const onScroll = () => {
      if (!updateScheduled) {
        updateScheduled = true;
        window.requestAnimationFrame(update);
      }
    };

    scroller.addEventListener('scroll', onScroll, { passive: true });

    // Smooth scroll for TOC clicks.
    toc.addEventListener('click', e => {
      const link = e.target.closest('a[href]');
      if (!link) return;
      const item = headings.find(h => h.link === link);
      if (!item) return;

      e.preventDefault();
      item.el.scrollIntoView({ behavior: 'smooth', block: 'start' });
      // Update URL without triggering a page reload
      history.replaceState(null, '', item.isPageTop ? window.location.pathname : `#${item.id}`);
      setActive(item.id);
    });

    update();
  }

  // Extract anchor IDs from href attributes.
  function extractIdFromHref(href) {
    if (!href || href === '#') return '';
    try {
      const url = new URL(href, window.location.href);
      return url.hash ? decodeURIComponent(url.hash.slice(1)) : '';
    } catch (_) {
      const i = href.indexOf('#');
      return i !== -1 ? decodeURIComponent(href.slice(i + 1)) : '';
    }
  }

  // Ensures the active TOC link remains visible within the sidebar if it overflows.
  function ensureLinkVisible(link, toc) {
    const container = toc;
    if (!container) return;
    const cRect = container.getBoundingClientRect();
    const lRect = link.getBoundingClientRect();
    if (lRect.top < cRect.top + 16) {
      container.scrollBy({ top: lRect.top - cRect.top - 16, behavior: 'smooth' });
    } else if (lRect.bottom > cRect.bottom - 16) {
      container.scrollBy({ top: lRect.bottom - cRect.bottom + 16, behavior: 'smooth' });
    }
  }

  // Initialization
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initRightToc);
  } else {
    initRightToc();
  }
})();