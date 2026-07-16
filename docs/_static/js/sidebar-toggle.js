document.addEventListener('DOMContentLoaded', () => {
  const arrowSvg = `<?xml version="1.0" encoding="UTF-8"?>
<svg viewBox="0 0 24 24" aria-hidden="true" focusable="false" class="sidebar-arrow">
  <path d="M8 4l8 8-8 8" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"/>
</svg>`;

  const stripIndexHtml = (path) => (
    path.endsWith('index.html') ? path.slice(0, -'index.html'.length) : path
  );

  const normalizePath = (rawPath) => {
    if (!rawPath) {
      return '';
    }
    try {
      return stripIndexHtml(new URL(rawPath, window.location.origin).pathname.replace(/^\/+/, ''));
    } catch (err) {
      return stripIndexHtml(String(rawPath).split(/[?#]/)[0].replace(/^\/+/, ''));
    }
  };

  const basenameOf = (rawPath) => normalizePath(rawPath).split('/').pop() || '';
  const matchesByBasename = (lhs, rhs) => {
    const left = basenameOf(lhs);
    const right = basenameOf(rhs);
    return !!left && !!right && left === right;
  };

  const pageName = document.body.getAttribute('data-page-name') || '';
  const currentPath = normalizePath(window.location.pathname);
  const currentBasename = currentPath.split('/').pop() || '';
  const pageMatches = (prefix) => (
    pageName.startsWith(prefix) || currentPath.includes(prefix) || currentPath.startsWith(prefix)
  );

  const onCppPage = pageMatches('cpp/api_cpp/');
  const onPythonApiPage = pageMatches('python/api_python/');
  const onPythonModulesPage = pageMatches('_modules/');
  const isCppNamespaceLanding = pageName.startsWith('cpp/api_cpp/namespace_') ||
    /^namespace_.*\.html$/.test(currentBasename);
  const isCppNamespacePage = isCppNamespaceLanding ||
    currentPath.includes('/cpp/api_cpp/namespace_') ||
    currentPath.startsWith('cpp/api_cpp/namespace_');
  const currentNamespace = onCppPage && isCppNamespacePage ? currentPath.split('#')[0] : null;

  if (onPythonApiPage || onPythonModulesPage || onCppPage) {
    document.body.classList.add('sidebar-show-level2-toggles');
  }

  const readStorageJson = (key, fallback) => {
    try {
      const raw = localStorage.getItem(key);
      if (!raw) {
        return fallback;
      }
      const parsed = JSON.parse(raw);
      return Array.isArray(parsed) ? parsed : fallback;
    } catch (err) {
      console.warn(`[sidebar-toggle] Failed to parse ${key}:`, err);
      return fallback;
    }
  };

  const OPEN_STATE_KEY = 'sidebarOpenSections';
  const openSections = new Set(readStorageJson(OPEN_STATE_KEY, []));
  const persistOpenSections = () => {
    try {
      localStorage.setItem(OPEN_STATE_KEY, JSON.stringify(Array.from(openSections)));
    } catch (err) {
      console.warn('[sidebar-toggle] Failed to persist sidebar state:', err);
    }
  };
  const exclusiveOpenSection = (key) => {
    openSections.clear();
    if (key) {
      openSections.add(key);
    }
    persistOpenSections();
  };

  const storedCppHrefRaw = localStorage.getItem('cppSidebarSelection');
  const storedCppHref = storedCppHrefRaw ? normalizePath(storedCppHrefRaw) : null;
  const storedCppOpenNamespaceRaw = localStorage.getItem('cppOpenNamespace');
  const storedCppOpenNamespace = storedCppOpenNamespaceRaw
    ? normalizePath(storedCppOpenNamespaceRaw)
    : null;

  const candidateNamespaceFromTokens = (tokens, availableNamespaces) => {
    if (!Array.isArray(tokens) || tokens.length < 3) {
      return null;
    }
    for (let end = tokens.length; end >= 3; end -= 1) {
      const candidate = `namespace_${tokens.slice(0, end).join('__')}.html`;
      if (!availableNamespaces || availableNamespaces.has(candidate)) {
        return candidate;
      }
    }
    return null;
  };

  const availableNamespaceHrefs = new Set(
    Array.from(document.querySelectorAll('.sidebar-links a.reference.internal[href]'))
      .map((link) => basenameOf(link.getAttribute('href') || ''))
      .filter((base) => /^namespace_.*\.html$/.test(base))
  );

  let effectiveNamespace = currentNamespace;
  const inferNamespaceFromPage = () => {
    if (!onCppPage) {
      return null;
    }

    const nsAnchor = document.querySelector('.content a.reference.internal[href*="namespace_"]');
    if (nsAnchor) {
      try {
        const resolved = new URL(nsAnchor.getAttribute('href'), window.location.href);
        const nsBase = basenameOf(resolved.pathname);
        if (nsBase) {
          return nsBase;
        }
      } catch (err) {
        console.warn('[sidebar-toggle] Failed to resolve namespace anchor:', err);
      }
    }

    const stem = currentBasename.replace(/\.html$/, '');
    const memberPrefixMatch = stem.match(/^(?:class|struct|enum|union|function|variable|typedef|define|file|dir)(.+)$/);
    if (memberPrefixMatch && memberPrefixMatch[1] && memberPrefixMatch[1].includes('_1_1')) {
      const tokens = memberPrefixMatch[1].split('_1_1').filter(Boolean);
      const byStem = candidateNamespaceFromTokens(tokens, availableNamespaceHrefs);
      if (byStem) {
        return byStem;
      }
    }

    const groupMatch = stem.match(/group__ouster__(?:sdk__)?([a-z0-9_]+?)__/i);
    if (groupMatch && groupMatch[1]) {
      const groupNs = `namespace_ouster__sdk__${groupMatch[1]}.html`;
      if (!availableNamespaceHrefs.size || availableNamespaceHrefs.has(groupNs)) {
        return groupNs;
      }
    }
    const camelGroupMatch = stem.match(/group__ouster([A-Za-z0-9]+)/i);
    if (camelGroupMatch && camelGroupMatch[1]) {
      const words = camelGroupMatch[1].match(/[A-Z]+(?=[A-Z][a-z]|$)|[A-Z]?[a-z]+|\d+/g) || [];
      if (words.length) {
        const groupNs = `namespace_ouster__sdk__${words[0].toLowerCase()}.html`;
        if (!availableNamespaceHrefs.size || availableNamespaceHrefs.has(groupNs)) {
          return groupNs;
        }
      }
    }

    const contentText = document.querySelector('.content')?.textContent || '';
    const symbolMatches = contentText.match(/ouster::sdk::[A-Za-z_][A-Za-z0-9_:]*/g) || [];
    for (const symbol of symbolMatches) {
      const tokens = symbol.split('::').filter(Boolean);
      const byContent = candidateNamespaceFromTokens(tokens, availableNamespaceHrefs);
      if (byContent) {
        return byContent;
      }
    }

    return null;
  };
  if (!effectiveNamespace) {
    effectiveNamespace = inferNamespaceFromPage();
  }

  if (effectiveNamespace) {
    localStorage.setItem('cppSidebarSelection', effectiveNamespace);
    localStorage.setItem('cppOpenNamespace', effectiveNamespace);
  }

  const isCppLink = (href, normalizedHref) => (
    normalizedHref.startsWith('cpp/api_cpp/') ||
    /^namespace_/.test(normalizedHref) ||
    /^function_/.test(normalizedHref) ||
    href.includes('cpp/api_cpp/')
  );

  const linkMatchesCurrent = (href, normalizedHref, isCpp) => {
    if (onPythonModulesPage && normalizedHref === 'python/api_python/ouster.sdk.html') {
      return true;
    }
    if (onCppPage && isCpp) {
      const currentFile = currentPath.split('/').pop();
      const hrefFile = href.split('/').pop();
      return currentFile === hrefFile ||
        (storedCppHref && storedCppHref.split('/').pop() === hrefFile);
    }
    return currentPath === normalizedHref || storedCppHref === normalizedHref;
  };

  document.querySelectorAll('.sidebar-links a').forEach((link) => {
    const href = link.getAttribute('href');
    if (!href) {
      return;
    }

    const normalizedHref = normalizePath(href);
    const cppLink = isCppLink(href, normalizedHref);
    if (cppLink) {
      link.addEventListener('click', () => {
        localStorage.setItem('cppSidebarSelection', normalizedHref);
      });
    }
    if (onCppPage) {
      const parentNamespaceLink = link.closest('li.toctree-l2')
        ?.querySelector(':scope > .sidebar-link-row > a.reference.internal[href]');
      if (parentNamespaceLink) {
        const parentNamespaceHref = normalizePath(parentNamespaceLink.getAttribute('href') || '');
        if (parentNamespaceHref) {
          link.addEventListener('click', () => {
            localStorage.setItem('cppOpenNamespace', parentNamespaceHref);
          });
        }
      }
    }

    const li = link.closest('li');
    if (!li) {
      return;
    }
    if (linkMatchesCurrent(href, normalizedHref, cppLink)) {
      li.classList.add('current');
      link.classList.add('current');
    }
  });

  const sectionEntries = [];
  const hasListChildren = (ul) => !!(ul && ul.childElementCount > 0);

  const findNestedList = (li) => {
    const lists = Array.from(li.querySelectorAll(':scope > ul'));
    const apiList = lists.find((ul) => ul.classList.contains('api-section-list')) || null;
    const preferApiSectionList = li.classList.contains('toctree-l2') && (
      onCppPage || onPythonApiPage || onPythonModulesPage
    );
    if (preferApiSectionList) {
      if (hasListChildren(apiList)) {
        return apiList;
      }
      const primary = lists.find((ul) => !ul.classList.contains('api-section-list'));
      return hasListChildren(primary) ? primary : null;
    }
    return lists.find((ul) => !ul.classList.contains('api-section-list')) || apiList;
  };

  const getLinkFromLi = (li) => {
    const row = li.querySelector(':scope > .sidebar-link-row');
    return li.querySelector(':scope > a') || row?.querySelector('a') || null;
  };

  const ensureLinkRow = (container, link) => {
    let row = container.querySelector(':scope > .sidebar-link-row');
    if (!row) {
      row = document.createElement('div');
      row.className = 'sidebar-link-row';
      container.insertBefore(row, link);
      row.appendChild(link);
    }
    return row;
  };

  const ensureToggleButton = (row, { label, labelPrefix = 'Toggle section', withHiddenLabel = true }) => {
    let btn = row.querySelector('button.sidebar-toggle');
    if (!btn) {
      btn = document.createElement('button');
      btn.type = 'button';
      btn.className = 'sidebar-toggle';
      btn.innerHTML = withHiddenLabel
        ? `<span class="visually-hidden">${labelPrefix}</span>${arrowSvg}`
        : arrowSvg;
      row.appendChild(btn);
    }
    if (label) {
      btn.setAttribute('aria-label', `${labelPrefix} ${label}`.trim());
    }
    return btn;
  };

  const setListExpanded = (button, list, open) => {
    button.setAttribute('aria-expanded', open ? 'true' : 'false');
    button.classList.toggle('is-open', open);
    list.classList.toggle('collapsed', !open);
  };

  let currentHash = window.location.hash || '';
  let lastHashActiveLink = null;
  let manualHashLockId = '';
  let manualHashLockUntil = 0;

  const scrollSidebarToLink = (link) => {
    if (!link) return;
    requestAnimationFrame(() => {
      // .vp-sidebar is the actual scrolling container (sidebar-links has overflow: visible)
      const sidebar = link.closest('.vp-sidebar') || link.closest('sidebar');
      if (!sidebar) return;
      const switcher = document.querySelector('.sidebar-version-switcher');
      const switcherHeight = switcher ? Math.ceil(switcher.getBoundingClientRect().height) + 8 : 16;
      const itemRect = link.getBoundingClientRect();
      const viewportRect = sidebar.getBoundingClientRect();
      if (itemRect.top < viewportRect.top + 16 ||
          itemRect.bottom > viewportRect.bottom - switcherHeight) {
        link.scrollIntoView({ block: 'nearest', behavior: 'smooth' });
      }
    });
  };

  const hashMatchesLink = (href) => {
    if (!href || !currentHash) {
      return false;
    }
    if (href.startsWith('#')) {
      return href === currentHash;
    }
    try {
      const resolved = new URL(href, window.location.href);
      return resolved.hash === currentHash;
    } catch (err) {
      return false;
    }
  };

  const HASH_ACTIVE_LINK_SELECTORS = [
    '.sidebar-links .api-section-list > li > ul a[href]',
    '.sidebar-links .api-section-list > li > a[href]',
    '.sidebar-links .api-section-list > li > .sidebar-link-row > a[href]',
  ];

  const groupToggleEntries = [];

  const applyApiGroupState = (groupLi, open) => {
    const childUl = groupLi?.querySelector(':scope > ul');
    const btn = groupLi?.querySelector(':scope > .sidebar-link-row > button.sidebar-toggle');
    if (!childUl || !btn) {
      return false;
    }
    setListExpanded(btn, childUl, open);
    return true;
  };

  const setupApiSectionGroupToggle = (groupLi) => {
    const childUl = groupLi.querySelector(':scope > ul');
    const groupLink = getLinkFromLi(groupLi);
    if (!childUl || !groupLink) {
      return;
    }

    // Landing-page groups (toctree-l1: ouster.sdk, namespace_ouster__sdk, …)
    // are purely navigational — the header link scrolls to a section and gets
    // highlighted; there is no expand/collapse toggle and the child list stays
    // hidden.  Subpackage/namespace pages (toctree-l2+) keep full toggle
    // behaviour.  Store the flag as a data attribute so setHashActiveLink can
    // read it without re-traversing the DOM.
    const parentLevelLi = groupLi.parentElement?.closest('li[class*="toctree-l"]');
    const isLanding = parentLevelLi?.classList.contains('toctree-l1') ?? false;
    if (isLanding) {
      groupLi.dataset.landingGroup = '';
    }

    const row = ensureLinkRow(groupLi, groupLink);
    const btn = isLanding ? null : ensureToggleButton(row, {
      label: (groupLink.textContent || '').trim(),
    });

    const groupHref = groupLink.getAttribute('href') || '';
    let userCollapsed = false;

    const syncFromLocation = () => {
      if (userCollapsed) {
        applyApiGroupState(groupLi, false);
        return;
      }
      const hasCurrentChild = !!childUl.querySelector('.current');
      const childMatches = Array.from(childUl.querySelectorAll('a[href]'))
        .some((a) => hashMatchesLink(a.getAttribute('href') || ''));
      // Landing pages: never open just because the header's own hash is active.
      const headerMatches = !isLanding && hashMatchesLink(groupHref);
      applyApiGroupState(groupLi, hasCurrentChild || childMatches || headerMatches);
    };

    syncFromLocation();
    groupToggleEntries.push({ sync: syncFromLocation, resetUserCollapsed: () => { userCollapsed = false; } });

    btn?.addEventListener('click', (event) => {
      event.preventDefault();
      event.stopPropagation();
      const isOpen = btn.getAttribute('aria-expanded') === 'true';
      userCollapsed = isOpen;
      applyApiGroupState(groupLi, !isOpen);
    });

    groupLink.addEventListener('click', (event) => {
      userCollapsed = false;
      if (!isLanding) {
        applyApiGroupState(groupLi, true);
      }
      // Scroll the content container to the target section and sync the
      // highlight directly.  Dispatching 'hashchange' would trigger the
      // scroll-spy before the smooth scroll reaches the target, overriding
      // the highlight with whichever section is currently at the viewport top.
      if (groupHref.startsWith('#')) {
        const targetEl = document.getElementById(groupHref.slice(1));
        if (targetEl) {
          event.preventDefault();
          manualHashLockId = groupHref.slice(1);
          manualHashLockUntil = performance.now() + 1200;
          targetEl.scrollIntoView({ block: 'start', behavior: 'smooth' });
          history.replaceState(null, '', groupHref);
          currentHash = groupHref;
          groupToggleEntries.forEach((e) => { e.resetUserCollapsed(); e.sync(); });
          setHashActiveLink(findHashActiveLink());
        }
      }
    });
  };

  document.querySelectorAll([
    '.sidebar-links li.toctree-l1 > ul.api-section-list > li',
    '.sidebar-links li.toctree-l2 > ul.api-section-list > li',
  ].join(', ')).forEach(setupApiSectionGroupToggle);

  const clearApiSectionHashActive = () => {
    document.querySelectorAll('.sidebar-links .api-section-list a.hash-active').forEach((link) => {
      link.classList.remove('hash-active');
    });
  };

  const findHashActiveLink = () => {
    for (const selector of HASH_ACTIVE_LINK_SELECTORS) {
      const match = Array.from(document.querySelectorAll(selector))
        .find((link) => hashMatchesLink(link.getAttribute('href') || ''));
      if (match) {
        return match;
      }
    }
    return null;
  };

  const setHashActiveLink = (activeLink) => {
    clearApiSectionHashActive();
    if (!activeLink) {
      lastHashActiveLink = null;
      return null;
    }
    activeLink.classList.add('hash-active');
    const groupLi = activeLink.closest('.api-section-list > li');
    // Auto-expand so the active child is visible — but not on landing-page
    // groups, where section headers are highlight-only (data-landing-group set
    // in setupApiSectionGroupToggle).
    if (groupLi && !('landingGroup' in groupLi.dataset)) {
      applyApiGroupState(groupLi, true);
    }
    if (activeLink !== lastHashActiveLink) {
      lastHashActiveLink = activeLink;
      scrollSidebarToLink(activeLink);
    }
    return activeLink;
  };

  const syncApiSectionHashActive = () => {
    currentHash = window.location.hash || '';
    if (!currentHash) {
      clearApiSectionHashActive();
      return null;
    }
    return setHashActiveLink(findHashActiveLink());
  };

  const extractHashId = (href) => {
    if (!href) {
      return '';
    }
    if (href.startsWith('#')) {
      return decodeURIComponent(href.slice(1));
    }
    try {
      const resolved = new URL(href, window.location.href);
      return resolved.hash ? decodeURIComponent(resolved.hash.slice(1)) : '';
    } catch (err) {
      const hashIndex = href.indexOf('#');
      return hashIndex === -1
        ? ''
        : decodeURIComponent(href.slice(hashIndex + 1));
    }
  };

  let scheduleApiSectionScrollUpdate = null;

  const setupApiSectionScrollSpy = () => {
    if (!onPythonApiPage && !onCppPage) {
      return;
    }
    const scroller = document.querySelector('.content-with-rail');
    if (!scroller) {
      return;
    }

    const spyItems = [];
    const seenIds = new Set();
    for (const selector of HASH_ACTIVE_LINK_SELECTORS) {
      document.querySelectorAll(selector).forEach((link) => {
        const id = extractHashId(link.getAttribute('href') || '');
        if (!id || seenIds.has(id)) {
          return;
        }
        const el = document.getElementById(id);
        if (!el) {
          return;
        }
        seenIds.add(id);
        spyItems.push({ link, id, el });
      });
    }
    if (!spyItems.length) {
      return;
    }

    spyItems.sort(
      (left, right) => left.el.getBoundingClientRect().top - right.el.getBoundingClientRect().top
    );

    let scrollScheduled = false;
    let scrollSpyUserScrolled = false;
    const updateFromScroll = () => {
      scrollScheduled = false;
      const triggerTop = Math.floor(window.innerHeight * 0.25);
      const scrollY = scroller.scrollTop;
      const isAtBottom = (scrollY + scroller.clientHeight) >= (scroller.scrollHeight - 25);

      if (manualHashLockId && performance.now() < manualHashLockUntil) {
        const lockedItem = spyItems.find((item) => item.id === manualHashLockId);
        if (lockedItem) {
          setHashActiveLink(lockedItem.link);
          const lockedHash = `#${lockedItem.id}`;
          if (window.location.hash !== lockedHash) {
            history.replaceState(
              null,
              '',
              `${window.location.pathname}${window.location.search}${lockedHash}`
            );
            currentHash = lockedHash;
          }
          return;
        }
      } else {
        manualHashLockId = '';
      }

      let candidateId = null;
      for (let index = spyItems.length - 1; index >= 0; index -= 1) {
        const item = spyItems[index];
        if (Math.round(item.el.getBoundingClientRect().top) <= triggerTop) {
          candidateId = item.id;
          break;
        }
      }

      const finalId = candidateId ||
        (isAtBottom && scrollY > 200
          ? spyItems[spyItems.length - 1].id
          : spyItems[0].id);
      const activeItem = spyItems.find((item) => item.id === finalId);
      if (!activeItem) {
        return;
      }

      // Before the user scrolls, keep a shared-link fragment that matches a spy
      // target instead of replacing it with the first sidebar section.
      if (!scrollSpyUserScrolled && window.location.hash) {
        const initialId = decodeURIComponent(window.location.hash.slice(1));
        const initialItem = spyItems.find((item) => item.id === initialId);
        if (initialItem) {
          setHashActiveLink(initialItem.link);
          return;
        }
      }

      setHashActiveLink(activeItem.link);
      const nextHash = `#${activeItem.id}`;
      if (window.location.hash !== nextHash) {
        history.replaceState(
          null,
          '',
          `${window.location.pathname}${window.location.search}${nextHash}`
        );
        currentHash = nextHash;
        groupToggleEntries.forEach((entry) => entry.sync());
      }
    };

    const scheduleScrollUpdate = () => {
      if (!scrollScheduled) {
        scrollScheduled = true;
        requestAnimationFrame(updateFromScroll);
      }
    };

    scroller.addEventListener('scroll', () => {
      scrollSpyUserScrolled = true;
      scheduleScrollUpdate();
    }, { passive: true });
    scheduleApiSectionScrollUpdate = scheduleScrollUpdate;
    scheduleScrollUpdate();
  };

  const onHashNavigation = () => {
    groupToggleEntries.forEach((entry) => {
      entry.resetUserCollapsed();
      entry.sync();
    });
    syncApiSectionHashActive();
    if (scheduleApiSectionScrollUpdate) {
      scheduleApiSectionScrollUpdate();
    }
  };

  window.addEventListener('hashchange', onHashNavigation);
  onHashNavigation();
  setupApiSectionScrollSpy();

  document.querySelectorAll('.sidebar-links li[class*="toctree-"]').forEach((li) => {
    const link = getLinkFromLi(li);
    if (!link) {
      return;
    }
    const nested = findNestedList(li);
    const levelClass = Array.from(li.classList).find((cls) => cls.startsWith('toctree-l')) || '';
    const isTopLevel = li.classList.contains('toctree-l1');
    const topLevelHref = normalizePath(link.getAttribute('href') || '');
    const label = (link.textContent || '').trim();
    const sectionKey = `${topLevelHref || (label ? `label:${label.toLowerCase().replace(/\s+/g, '-')}` : '')}|${levelClass}`;

    if (isTopLevel) {
      link.addEventListener('click', () => {
        exclusiveOpenSection(sectionKey || topLevelHref || null);
      });
    }

    const row = ensureLinkRow(li, link);
    if (nested) {
      const btn = ensureToggleButton(row, { label });

      const hasCurrentChild = !!nested.querySelector('.current');
      const isCppSection = label === 'C++ API Reference';
      const isPythonModulesSection = topLevelHref === 'python/api_python/ouster.sdk.html';
      const defaultOpen = li.classList.contains('current') || hasCurrentChild;
      const shouldExpand = (
        (isTopLevel && openSections.has(sectionKey)) ||
        (onCppPage && isCppSection) ||
        (onCppPage && li.classList.contains('toctree-l2') &&
          storedCppOpenNamespace && matchesByBasename(topLevelHref, storedCppOpenNamespace)) ||
        (onCppPage && li.classList.contains('toctree-l2') &&
          effectiveNamespace && matchesByBasename(topLevelHref, effectiveNamespace)) ||
        defaultOpen ||
        (onPythonApiPage && isPythonModulesSection) ||
        (onPythonModulesPage && isPythonModulesSection)
      );
      const fallbackExpanded = btn.getAttribute('aria-expanded') === 'true' && !defaultOpen;

      const recordState = (open, { exclusive = false } = {}) => {
        if (!isTopLevel || !sectionKey) {
          return;
        }
        if (exclusive && open) {
          exclusiveOpenSection(sectionKey);
          return;
        }
        if (open) {
          openSections.add(sectionKey);
        } else {
          openSections.delete(sectionKey);
        }
        persistOpenSections();
      };

      const applyState = (open, { persist = false, exclusive = false } = {}) => {
        setListExpanded(btn, nested, open);
        if (persist) {
          recordState(open, { exclusive });
        }
      };

      const entry = {
        key: sectionKey,
        apply: (open, opts) => applyState(open, opts),
      };
      if (isTopLevel) {
        sectionEntries.push(entry);
      }

      applyState(shouldExpand || fallbackExpanded);
      if (isTopLevel && defaultOpen && shouldExpand) {
        exclusiveOpenSection(sectionKey);
      }

      btn.addEventListener('click', (event) => {
        event.preventDefault();
        event.stopPropagation();
        const open = btn.getAttribute('aria-expanded') === 'true';
        if (isTopLevel) {
          if (open) {
            applyState(false, { persist: true });
          } else {
            sectionEntries.forEach((candidate) => {
              if (candidate !== entry) {
                candidate.apply(false, { persist: true });
              }
            });
            applyState(true, { persist: true, exclusive: true });
          }
        } else {
          applyState(!open);
        }
      });

      if (!isTopLevel) {
        link.addEventListener('click', () => {
          if (btn.getAttribute('aria-expanded') !== 'true') {
            applyState(true);
          }
        });
      }
    } else {
      const btn = ensureToggleButton(row, {
        label,
        labelPrefix: 'Open',
        withHiddenLabel: false,
      });
      btn.classList.add('no-children');
      btn.addEventListener('click', (event) => {
        event.preventDefault();
        event.stopPropagation();
        link.click();
      });
    }
  });

  const scrollIntoViewIfNeeded = (element) => {
    if (!element) {
      return;
    }
    // .vp-sidebar is the actual scrolling container
    const sidebarViewport = element.closest('.vp-sidebar') || element.closest('sidebar');
    if (!sidebarViewport) {
      return;
    }
    const switcher = document.querySelector('.sidebar-version-switcher');
    const switcherHeight = switcher ? Math.ceil(switcher.getBoundingClientRect().height) + 8 : 16;
    const itemRect = element.getBoundingClientRect();
    const viewportRect = sidebarViewport.getBoundingClientRect();
    if (itemRect.top < viewportRect.top || itemRect.bottom > viewportRect.bottom - switcherHeight) {
      element.scrollIntoView({ block: 'nearest', inline: 'nearest' });
    }
  };

  requestAnimationFrame(() => {
    const currentLinks = Array.from(document.querySelectorAll('.sidebar-links a.current'));
    scrollIntoViewIfNeeded(
      document.querySelector('.sidebar-links .api-section-list a.hash-active') ||
      currentLinks.find((link) => link.closest('ul.api-section-list')) ||
      currentLinks[currentLinks.length - 1]
    );
  });
});
