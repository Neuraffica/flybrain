/* Offline Bittle artwork; see NOTICE. Loaded only by dashboards with a robot view. */
(() => {
  const base = new URL('.', document.currentScript.src);
  const images = new Map();
  const reduced = matchMedia('(prefers-reduced-motion: reduce)');
  const image = (name) => {
    if (!images.has(name)) {
      const img = new Image();
      img.src = new URL(name + '.webp', base).href;
      images.set(name, img);
    }
    return images.get(name);
  };
  const frame = (pose, phase) => pose === 'rest' ? 1 :
    !reduced.matches && ['back', 'walk', 'turn'].includes(pose) ? 2 + Math.floor(((phase % 1 + 1) % 1) * 8) : 0;
  const tile = (g, name, index, x, y, size) => {
    const img = image(name);
    if (!img.complete || !img.naturalWidth) return;
    const cell = img.naturalHeight;
    g.drawImage(img, index * cell, 0, cell, cell, x, y, size, size);
  };
  window.BittleArt = {
    top(g, scale, pose = 'stand', phase = 0, pan = 0) {
      g.save(); g.scale(scale, scale);
      tile(g, 'top', frame(pose, phase), -16, -7.15, 32);
      // Neck pivot from the model; pan comes only from the commanded joint.
      g.translate(0, 4.095);
      g.rotate(-pan * Math.PI / 180);
      tile(g, 'head', 0, -16, -11.245, 32);
      g.restore();
    },
    side(g, scale, pose = 'stand', phase = 0) {
      // Origin is sensor x / floor y. Shared view framing is 32 cm square.
      tile(g, 'side', frame(pose, phase), -24.85 * scale, -30 * scale, 32 * scale);
    },
  };
})();
