document.addEventListener('DOMContentLoaded', function () {
  const selector = 'div.highlight pre';
  document.querySelectorAll(selector).forEach(function (pre) {
    const lines = pre.innerHTML.split(/\r?\n/);
    let changed = false;
    const filtered = lines.filter(function (line) {
      if (line.includes('//! [doc')) {
        changed = true;
        return false;
      }
      if (line.includes('// [doc')) {
        changed = true;
        return false;
      }
      if (line.includes('# [doc')) {
        changed = true;
        return false;
      }
      return true;
    });
    if (changed) {
      pre.innerHTML = filtered.join('\n');
    }
  });
});
