(function () {
  function cssEscape(value) {
    if (window.CSS && typeof window.CSS.escape === "function") {
      return window.CSS.escape(value);
    }
    return value.replace(/["\\]/g, "\\$&");
  }

  function findHashTarget(hash) {
    if (!hash || hash === "#") return null;
    const id = decodeURIComponent(hash.slice(1));
    if (!id) return null;
    return document.getElementById(id) || document.querySelector('[id="' + cssEscape(id) + '"]');
  }

  function isApiReferencePage() {
    const path = window.location.pathname || "";
    return path.includes("/python/api_python/") || path.includes("/cpp/api_cpp/");
  }

  function settleAnchorScroll() {
    if (!isApiReferencePage()) return;
    const hash = window.location.hash;
    if (!hash || hash === "#") return;

    const delays = [0, 50, 150, 300, 600];
    delays.forEach((delay) => {
      window.setTimeout(() => {
        if (window.location.hash !== hash) return;
        const target = findHashTarget(hash);
        if (target && typeof target.scrollIntoView === "function") {
          target.scrollIntoView({ block: "start" });
        }
      }, delay);
    });
  }

  function datasetKey(lang) {
    if (!lang) return "";
    return "ousterTarget" + lang.charAt(0).toUpperCase() + lang.slice(1);
  }

  function currentLanguageFor(group) {
    if (!group) return "py";
    const selector = '.sd-tab-set input:checked + label[data-sync-group="' + group + '"]';
    const activeLabel = document.querySelector(selector);
    if (activeLabel && activeLabel instanceof HTMLElement) {
      return activeLabel.getAttribute("data-sync-id") || "py";
    }
    try {
      const stored = window.sessionStorage.getItem("sphinx-design-tab-id-" + group);
      if (stored) {
        return stored;
      }
    } catch (err) {
      /* ignore sessionStorage issues */
    }
    return "py";
  }

  function syncLinks() {
    const anchors = document.querySelectorAll("a.ouster-tabbed-api-link");
    if (!anchors.length) {
      return;
    }

    anchors.forEach((anchor) => {
      syncOneLink(anchor);
    });
  }

  function syncOneLink(anchor) {
    const group = anchor.dataset.ousterSyncGroup || "api-lang";
    const lang = currentLanguageFor(group);
    const key = datasetKey(lang);
    const fallbackKey = datasetKey(anchor.dataset.ousterDefaultLang || "py");
    const rawHref = (key && anchor.dataset[key]) || anchor.dataset[fallbackKey];
    if (rawHref) {
      // Keep href as authored (relative + hash) and let the browser navigate natively.
      anchor.setAttribute("href", rawHref);
      anchor.setAttribute("target", "_blank");
      anchor.setAttribute("rel", "noopener");
    }
    anchor.dataset.ousterActiveLang = lang;
  }

  function attachListeners() {
    const labels = document.querySelectorAll('.sd-tab-label[data-sync-group]');
    const schedule = () => window.requestAnimationFrame(syncLinks);
    labels.forEach((label) => {
      label.addEventListener("click", schedule);
      label.addEventListener("keyup", (event) => {
        if (event.key === "Enter" || event.key === " ") {
          schedule();
        }
      });
    });

    // Firefox updates radio state reliably on `change`; listen here as well.
    const radios = document.querySelectorAll('.sd-tab-set input[type="radio"]');
    radios.forEach((radio) => {
      radio.addEventListener("change", schedule);
    });

    // Update href immediately before native navigation.
    const syncFromEvent = (event) => {
      const anchor = event.target && event.target.closest
        ? event.target.closest("a.ouster-tabbed-api-link")
        : null;
      if (anchor instanceof HTMLAnchorElement) {
        syncOneLink(anchor);
      }
    };
    document.addEventListener("mousedown", syncFromEvent, true);
    document.addEventListener("touchstart", syncFromEvent, true);
    document.addEventListener("focusin", syncFromEvent, true);
  }

  document.addEventListener("DOMContentLoaded", () => {
    syncLinks();
    attachListeners();
    settleAnchorScroll();
  });

  window.addEventListener("load", settleAnchorScroll);
  window.addEventListener("hashchange", settleAnchorScroll);
})();
