// A few nav entries point away from this site (the Doxygen API reference,
// the two GitHub repos) — open those in their own tab instead of navigating
// away from the docs.
(function () {
  var EXTERNAL_HREFS = [
    "/doxygen/",
    "https://github.com/Software-Solaris/solaris-software",
    "https://github.com/Software-Solaris/solaris-packet-protocol",
  ];

  window.document$.subscribe(function () {
    EXTERNAL_HREFS.forEach(function (href) {
      document.querySelectorAll('a[href="' + href + '"]').forEach(function (a) {
        a.target = "_blank";
        a.rel = "noopener";
      });
    });
  });
})();
