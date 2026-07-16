(function() {
  function initRightTocToggle() {
    var toc = document.querySelector('.right-toc');
    if (!toc) {
      return;
    }

    var items = toc.querySelectorAll('li');
    if (items.length <= 1) {
      var sidebar = toc.closest('.right-sidebar');
      if (sidebar) {
        sidebar.style.display = 'none';
      }
      toc.remove();
    }
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initRightTocToggle);
  } else {
    initRightTocToggle();
  }
})();
