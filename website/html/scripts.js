(function () {
    const lines = [
        "Solaris Software"
    ];

    const el = document.getElementById('heroTerminal');
    let li = 0, ci = 0;
    const done = [];

    function render() {
        const finished = li >= lines.length;
        let html = done.map((l, i) =>
            (finished && i === done.length - 1) ? l : l + '<br>'
        ).join('');
        if (!finished) html += lines[li].slice(0, ci);
        html += '<span class="cur"></span>';
        el.innerHTML = html;
    }

    function tick() {
        if (li >= lines.length) return;
        if (ci < lines[li].length) {
            ci++;
            render();
            setTimeout(tick, 60 + Math.random() * 60);
        } else {
            done.push(lines[li]);
            li++; ci = 0;
            render();
            if (li < lines.length) setTimeout(tick, 700);
        }
    }

    render();
    setTimeout(tick, 1000);
})();

(function () {
    const CHARS = '01!@#%<>[]{}?/\\|=+-*^~$&';

    function scramble(el) {
        const walker = document.createTreeWalker(el, NodeFilter.SHOW_TEXT);
        const nodes = [];
        let n;
        while ((n = walker.nextNode())) {
            if (n.textContent.trim()) nodes.push([n, n.textContent]);
        }
        nodes.forEach(([node, original]) => {
            let iter = 0;
            const iv = setInterval(() => {
                node.textContent = original.split('').map((ch, i) => {
                    if (ch === ' ') return ch;
                    if (i < iter) return original[i];
                    return CHARS[Math.floor(Math.random() * CHARS.length)];
                }).join('');
                iter += 0.45;
                if (iter > original.length) {
                    node.textContent = original;
                    clearInterval(iv);
                }
            }, 22);
        });
    }

    document.querySelectorAll('.stack, .team, .rl').forEach(group => {
        Array.from(group.children).forEach((child, i) => {
            child.classList.add('reveal');
            child.style.transitionDelay = `${i * 0.07}s`;
        });
    });

    document.querySelectorAll(
        '.body-text, .hero-copy, .hero-actions, .hero-meta, .hero-rule, .sponsor-box, .principle-rule, .principle-sub'
    ).forEach(el => el.classList.add('reveal'));

    document.querySelectorAll('.s-title, .cmd, .principle-text').forEach(el => {
        el.classList.add('reveal');
        el.dataset.scrambled = '0';
    });

    const revealObs = new IntersectionObserver((entries) => {
        entries.forEach(entry => {
            if (!entry.isIntersecting) return;
            const element = entry.target;
            element.classList.add('visible');
            if (element.dataset.scrambled === '0') {
                element.dataset.scrambled = '1';
                setTimeout(() => scramble(element), 80);
            }
            revealObs.unobserve(element);
        });
    }, { threshold: 0.15 });

    document.querySelectorAll('.reveal').forEach(el => revealObs.observe(el));
})();

(function () {
    const canvas = document.getElementById('matrixCanvas');
    const ctx = canvas.getContext('2d');
    const CHARS = 'アイウエオカキクケコサシスセソタチツテト0123456789!#$%<>[]{}01';
    const COL_W = 12;
    let drops = [];

    const TRAIL = 18;
    const SPEED = 0.35;
    let charBufs = [];

    function resize() {
        const hero = canvas.parentElement;
        canvas.width  = hero.offsetWidth;
        canvas.height = hero.offsetHeight;
        const cols = Math.ceil(canvas.width / COL_W);
        drops = Array.from({ length: cols }, () => Math.random() * (canvas.height / COL_W));
        charBufs = Array.from({ length: cols }, () =>
            Array.from({ length: TRAIL + 1 }, () => CHARS[Math.floor(Math.random() * CHARS.length)])
        );
    }

    resize();
    window.addEventListener('resize', resize);

    function draw() {
        ctx.clearRect(0, 0, canvas.width, canvas.height);

        for (let i = 0; i < drops.length; i++) {
            const head = Math.floor(drops[i]);

            for (let t = TRAIL; t >= 1; t--) {
                const row = head - t;
                if (row < 0) continue;
                const alpha = (1 - t / TRAIL) * 0.65;
                ctx.fillStyle = `rgba(210, 22, 38, ${alpha})`;
                ctx.font = `${COL_W - 3}px monospace`;
                ctx.fillText(charBufs[i][t], i * COL_W, row * COL_W);
            }

            charBufs[i][0] = CHARS[Math.floor(Math.random() * CHARS.length)];
            ctx.fillStyle = 'rgba(255, 75, 85, 0.95)';
            ctx.font = `bold ${COL_W - 2}px monospace`;
            ctx.fillText(charBufs[i][0], i * COL_W, head * COL_W);

            drops[i] += SPEED;
            if (head * COL_W > canvas.height) {
                drops[i] = Math.random() * -TRAIL;
                charBufs[i] = Array.from({ length: TRAIL + 1 }, () => CHARS[Math.floor(Math.random() * CHARS.length)]);
            }
        }
    }

    setInterval(draw, 40);
})();

(function () {
    let started = false;

    function startAmbient() {
        if (started) return;
        started = true;

        const ctx = new (window.AudioContext || window.webkitAudioContext)();
        ctx.resume();

        const master = ctx.createGain();
        master.gain.setValueAtTime(0, ctx.currentTime);
        master.gain.linearRampToValueAtTime(0.18, ctx.currentTime + 4);
        master.connect(ctx.destination);

        const bufSize = ctx.sampleRate * 3;
        const buf = ctx.createBuffer(1, bufSize, ctx.sampleRate);
        const data = buf.getChannelData(0);
        for (let i = 0; i < bufSize; i++) data[i] = Math.random() * 2 - 1;
        const noise = ctx.createBufferSource();
        noise.buffer = buf;
        noise.loop = true;
        const lp = ctx.createBiquadFilter();
        lp.type = 'lowpass';
        lp.frequency.value = 200;
        lp.Q.value = 0.8;
        const noiseGain = ctx.createGain();
        noiseGain.gain.value = 0.4;
        noise.connect(lp);
        lp.connect(noiseGain);
        noiseGain.connect(master);
        noise.start();

        const osc1 = ctx.createOscillator();
        osc1.type = 'sine';
        osc1.frequency.value = 60;
        const g1 = ctx.createGain();
        g1.gain.value = 0.7;
        osc1.connect(g1);
        g1.connect(master);
        osc1.start();

        const osc2 = ctx.createOscillator();
        osc2.type = 'sine';
        osc2.frequency.value = 120;
        const g2 = ctx.createGain();
        g2.gain.value = 0.25;
        osc2.connect(g2);
        g2.connect(master);
        osc2.start();

        const lfo = ctx.createOscillator();
        lfo.type = 'sine';
        lfo.frequency.value = 0.08;
        const lfoGain = ctx.createGain();
        lfoGain.gain.value = 0.04;
        lfo.connect(lfoGain);
        lfoGain.connect(master.gain);
        lfo.start();
    }

    ['scroll', 'click', 'keydown', 'touchstart'].forEach(ev =>
        window.addEventListener(ev, startAmbient, { once: false, passive: true })
    );
})();
