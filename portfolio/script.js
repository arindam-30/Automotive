// In normal layout the page itself scrolls; in "force-landscape" desktop
// view (see bottom of file) body becomes the scroll container instead, so
// scroll position/height are read generically to cover both.
const getScrollTop = () => window.scrollY || document.documentElement.scrollTop || document.body.scrollTop || 0;
const getScrollableHeight = () => {
  const doc = document.documentElement;
  const totalHeight = Math.max(doc.scrollHeight, document.body.scrollHeight);
  const viewport = document.documentElement.classList.contains('force-landscape')
    ? window.innerWidth
    : window.innerHeight;
  return totalHeight - viewport;
};

// Scroll progress bar: fills as the visitor moves through the deck
const progressFill = document.getElementById('scrollProgressFill');

const updateScrollProgress = () => {
  const scrollableHeight = getScrollableHeight();
  const pct = scrollableHeight > 0 ? (getScrollTop() / scrollableHeight) * 100 : 0;
  progressFill.style.width = `${pct}%`;
};
updateScrollProgress();
document.addEventListener('scroll', updateScrollProgress, { passive: true, capture: true });
window.addEventListener('resize', updateScrollProgress);

// Scroll-reveal: fade/slide in elements as they enter the viewport
const revealEls = document.querySelectorAll('.reveal');

const revealObserver = new IntersectionObserver((entries) => {
  entries.forEach((entry) => {
    entry.target.classList.toggle('is-visible', entry.isIntersecting);
  });
}, { threshold: 0.2 });

revealEls.forEach((el) => revealObserver.observe(el));

// Sticky nav: add shadow/background once the page has scrolled
const nav = document.querySelector('.nav');

const setNavScrolled = () => {
  nav.classList.toggle('is-scrolled', getScrollTop() > 8);
};
setNavScrolled();
document.addEventListener('scroll', setNavScrolled, { passive: true, capture: true });

// Mobile nav toggle
const navToggle = document.getElementById('navToggle');
const navLinks = document.querySelector('.nav-links');

navToggle.addEventListener('click', () => {
  const isOpen = navLinks.classList.toggle('is-open');
  navToggle.setAttribute('aria-expanded', String(isOpen));
});

navLinks.querySelectorAll('a').forEach((link) => {
  link.addEventListener('click', () => {
    navLinks.classList.remove('is-open');
    navToggle.setAttribute('aria-expanded', 'false');
  });
});

// Project detail overlay: opened from the "My Projects" heading or any project card
const projectOverlay = document.getElementById('projectOverlay');
const projectOverlayScroll = document.getElementById('projectOverlayScroll');
const projectOverlayClose = document.getElementById('projectOverlayClose');
const projectOpenTriggers = document.querySelectorAll('[data-open-project]');

const openProjectOverlay = (targetId) => {
  projectOverlay.classList.add('is-open');
  projectOverlay.setAttribute('aria-hidden', 'false');
  document.documentElement.classList.add('no-scroll');
  const target = targetId ? document.getElementById(targetId) : null;
  if (target) {
    target.scrollIntoView({ behavior: 'instant', block: 'start' });
  } else {
    projectOverlayScroll.scrollTop = 0;
  }
};

const closeProjectOverlay = () => {
  projectOverlay.classList.remove('is-open');
  projectOverlay.setAttribute('aria-hidden', 'true');
  document.documentElement.classList.remove('no-scroll');
};

projectOpenTriggers.forEach((el) => {
  el.addEventListener('click', () => openProjectOverlay(el.dataset.openProject));
  el.addEventListener('keydown', (event) => {
    if (event.key === 'Enter' || event.key === ' ') {
      event.preventDefault();
      openProjectOverlay(el.dataset.openProject);
    }
  });
});

projectOverlayClose.addEventListener('click', closeProjectOverlay);

document.addEventListener('keydown', (event) => {
  if (event.key === 'Escape' && projectOverlay.classList.contains('is-open')) {
    closeProjectOverlay();
  }
});

// Certificate detail overlay: same pattern as the project overlay, opened
// from the "Certificates" heading or any certificate card
const certOverlay = document.getElementById('certOverlay');
const certOverlayScroll = document.getElementById('certOverlayScroll');
const certOverlayClose = document.getElementById('certOverlayClose');
const certOpenTriggers = document.querySelectorAll('[data-open-cert]');

const openCertOverlay = (targetId) => {
  certOverlay.classList.add('is-open');
  certOverlay.setAttribute('aria-hidden', 'false');
  document.documentElement.classList.add('no-scroll');
  const target = targetId ? document.getElementById(targetId) : null;
  if (target) {
    target.scrollIntoView({ behavior: 'instant', block: 'start' });
  } else {
    certOverlayScroll.scrollTop = 0;
  }
};

const closeCertOverlay = () => {
  certOverlay.classList.remove('is-open');
  certOverlay.setAttribute('aria-hidden', 'true');
  document.documentElement.classList.remove('no-scroll');
};

certOpenTriggers.forEach((el) => {
  el.addEventListener('click', () => openCertOverlay(el.dataset.openCert));
  el.addEventListener('keydown', (event) => {
    if (event.key === 'Enter' || event.key === ' ') {
      event.preventDefault();
      openCertOverlay(el.dataset.openCert);
    }
  });
});

certOverlayClose.addEventListener('click', closeCertOverlay);

document.addEventListener('keydown', (event) => {
  if (event.key === 'Escape' && certOverlay.classList.contains('is-open')) {
    closeCertOverlay();
  }
});

// "Switch to Desktop View" for mobile visitors: disables the mobile
// stylesheet outright (so every @media(max-width) override drops out no
// matter how narrow the real viewport is) and rotates the page 90deg so
// the full desktop layout sits in a landscape frame, like a desktop
// browser window, without needing the phone physically rotated.
const mobileStylesLink = document.getElementById('mobileStyles');
const desktopViewToggle = document.getElementById('desktopViewToggle');
const desktopViewExit = document.getElementById('desktopViewExit');

// The desktop layout (3-column grids, wide padding) needs real room to look
// right, but the rotated screen width (a phone's own portrait height) is
// usually narrower than that. Render the rotated page against a fixed
// reference-width canvas, sized so that scaling it down by `scale` reproduces
// exactly the same final rotated bounding box the un-scaled version used
// (real device height x real device width) -- same fit, just with desktop-
// sized layout internally instead of content overflowing a too-narrow box.
const LANDSCAPE_REFERENCE_WIDTH = 1024;

const updateLandscapeZoom = () => {
  if (!document.documentElement.classList.contains('force-landscape')) return;
  const vw = window.innerWidth;
  const vh = window.innerHeight;
  const scale = Math.min(1, Math.max(0.5, vh / LANDSCAPE_REFERENCE_WIDTH));
  const height = vw / scale;
  document.documentElement.style.setProperty('--landscape-scale', scale);
  document.documentElement.style.setProperty('--landscape-width', `${LANDSCAPE_REFERENCE_WIDTH}px`);
  document.documentElement.style.setProperty('--landscape-height', `${height}px`);
};

const enterDesktopView = () => {
  if (mobileStylesLink.sheet) mobileStylesLink.sheet.disabled = true;
  mobileStylesLink.disabled = true;
  document.documentElement.classList.add('force-landscape');
  navLinks.classList.remove('is-open');
  navToggle.setAttribute('aria-expanded', 'false');
  updateLandscapeZoom();
  window.scrollTo(0, 0);
  document.body.scrollTop = 0;
  updateScrollProgress();
};

const exitDesktopView = () => {
  mobileStylesLink.disabled = false;
  if (mobileStylesLink.sheet) mobileStylesLink.sheet.disabled = false;
  document.documentElement.classList.remove('force-landscape');
  document.documentElement.style.removeProperty('--landscape-scale');
  document.documentElement.style.removeProperty('--landscape-width');
  document.documentElement.style.removeProperty('--landscape-height');
  window.scrollTo(0, 0);
  document.body.scrollTop = 0;
  updateScrollProgress();
};

desktopViewToggle.addEventListener('click', enterDesktopView);
desktopViewExit.addEventListener('click', exitDesktopView);
window.addEventListener('resize', updateLandscapeZoom);
