(function () {
    if (document.readyState === "loading") {
      document.addEventListener("DOMContentLoaded", main);
    } else {
      main();
    }
  
    function main() {
      if (!/\/cpp\/api_cpp\//.test(window.location.pathname)) return;
  
      const HIDE_PATTERNS = [
        /ChanField/,         // matches ChanField anywhere
      ];
  
      const links = document.querySelectorAll(
        ".wy-menu-vertical a, .toctree-wrapper a"
      );
      links.forEach(link => {
        if (link.dataset.cppNavClean === "1") return;
  
        const text = link.textContent || "";
        const href = link.getAttribute("href") || "";
  
        if (shouldHide(text, href)) {
          const li = link.closest("li");
          if (li) {
            li.remove();
          } else {
            link.remove();
          }
          return;
        }
  
        if (text.startsWith("Namespace ")) {
          link.textContent = text.replace(/^Namespace\s+/, "");
        }
  
        link.dataset.cppNavClean = "1";
      });
  
      function shouldHide(text, href) {
        return HIDE_PATTERNS.some(rx => rx.test(text) || rx.test(href));
      }
    }
  })();