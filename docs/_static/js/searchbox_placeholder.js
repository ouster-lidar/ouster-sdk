(function () {
  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", init);
  } else {
    init();
  }

  function init() {
    var searchInput = document.querySelector('#searchbox input[name="q"]');
    if (!searchInput) {
      return;
    }

    if (!searchInput.getAttribute('placeholder')) {
      searchInput.setAttribute('placeholder', 'Search');
    }

    var submitButton = document.querySelector('#searchbox input[type="submit"]');
    if (submitButton) {
      submitButton.style.display = 'none';
    }

    searchInput.addEventListener('keydown', function (event) {
      if (event.key === 'Enter') {
        event.preventDefault();
        if (searchInput.form) {
          searchInput.form.submit();
        }
      }
    });
  }
})();
