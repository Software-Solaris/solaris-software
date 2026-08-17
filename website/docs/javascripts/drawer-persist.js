// Keep the mobile nav drawer open across page navigation.
//
// Material for MkDocs closes the drawer itself 125ms after every navigation
// (see their bundle: `merge(location$, target$).pipe(delay(125)).subscribe(
// () => setToggle("drawer", false))`). We hook into the same `location$`
// event, remember whether the drawer was open at that moment, and re-open it
// slightly after their own timer fires — so it only closes when the user
// taps the toggle (or the overlay) themselves.
(function () {
  window.location$.subscribe(function () {
    var drawer = document.getElementById("__drawer");
    var wasOpen = !!(drawer && drawer.checked);
    if (!wasOpen) {
      return;
    }

    setTimeout(function () {
      if (drawer && !drawer.checked) {
        // Use a real click, like Material does, so its own toggle
        // listeners (scroll lock, focus handling, etc.) stay in sync.
        drawer.click();
      }
    }, 150);
  });
})();
