(function () {
  // TODO: Branch name hardcoded for now, switch to using env var maybe ?
  var GITLAB_BASE = 'https://gitlab.com/ouster/software/ouster-sdk/-/blob/develop/';
  var GITHUB_BASE = 'https://github.com/ouster-lidar/ouster-sdk/blob/master/';
  var isInternal = Boolean(window.DOCS_IS_INTERNAL);
  var ACTIVE_BASE = isInternal ? GITLAB_BASE : GITHUB_BASE;
  var BUTTON_LABEL = isInternal ? 'View on GitLab' : 'View on GitHub';
  var BUTTON_TOOLTIP = isInternal ? 'View on GitLab' : 'View on GitHub';

  function initGithubButtons() {
    document.querySelectorAll('.doc-snippet').forEach(function (block) {
      var wrapper = block.closest('.literal-block-wrapper');
      var caption = wrapper
        ? wrapper.querySelector(':scope > .code-block-caption')
        : block.previousElementSibling;
      if (!caption || !caption.classList.contains('code-block-caption')) {
        return;
      }
      if (caption.dataset.githubButtonProcessed === '1') {
        return;
      }

      var captionLink = null;
      var possibleLinks = caption.querySelectorAll('a[href]');
      possibleLinks.forEach(function (link) {
        if (captionLink) {
          return;
        }
        var href = link.getAttribute('href');
        if (href && href.indexOf('|github-src|') === 0) {
          captionLink = link;
        }
      });
      if (!captionLink && possibleLinks.length) {
        captionLink = possibleLinks[0];
      }
      if (!captionLink) {
        return;
      }

      var highlight = block.querySelector('.highlight') || block;
      if (highlight.querySelector('.githubbtn')) {
        return;
      }

      var href = captionLink.getAttribute('href');
      if (href && href.indexOf('|github-src|') === 0) {
        href = href.replace('|github-src|', ACTIVE_BASE);
        captionLink.setAttribute('href', href);
      }

      var button = document.createElement('button');
      button.className = 'githubbtn o-tooltip--left';
      button.type = 'button';
      button.setAttribute('data-tooltip', BUTTON_TOOLTIP);
      button.setAttribute('aria-label', BUTTON_LABEL);
      button.innerHTML = '\n      <svg xmlns="http://www.w3.org/2000/svg" class="icon icon-tabler icon-tabler-brand-github" width="44" height="44" viewBox="0 0 24 24" stroke-width="1.5" stroke="currentColor" fill="none" stroke-linecap="round" stroke-linejoin="round">\n        <path stroke="none" d="M0 0h24v24H0z" fill="none"></path>\n        <path d="M9 19c-4.2 1.4 -4.2 -2.4 -6 -2.8" />\n        <path d="M15 21v-3.5c0 -1 .1 -1.4 -.5 -2c2.8 -.3 5.5 -1.4 5.5 -6a4.6 4.6 0 0 0 -1.3 -3.2a4.2 4.2 0 0 0 -.1 -3.2s-1.1 -.3 -3.5 1.3a12.3 12.3 0 0 0 -6.2 0c-2.4 -1.6 -3.5 -1.3 -3.5 -1.3a4.2 4.2 0 0 0 -.1 3.2a4.6 4.6 0 0 0 -1.3 3.2c0 4.6 2.7 5.7 5.5 6c-.6 .6 -.6 1.2 -.5 2v3.5" />\n      </svg>\n      ';
      button.addEventListener('click', function (event) {
        event.preventDefault();
        window.open(captionLink.href, '_blank', 'noopener');
      });

      highlight.appendChild(button);
      caption.style.display = 'none';
      caption.dataset.githubButtonProcessed = '1';
    });
  }

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initGithubButtons);
  } else {
    initGithubButtons();
  }
})();
