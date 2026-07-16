(function () {
  function markReady() {
    document.documentElement.classList.remove('page-loading');
    document.documentElement.classList.add('page-ready');
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', function () {
      window.requestAnimationFrame(markReady);
    });
  } else {
    window.requestAnimationFrame(markReady);
  }
})();
