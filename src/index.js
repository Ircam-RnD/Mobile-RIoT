import * as lfo from 'waves-lfo/client';
import * as lfoMotion from 'lfo-motion';
import * as audio from 'waves-audio';
import * as loaders from 'waves-loaders';
import riot from './riot';
import View from './View';
import AudioEngine from './AudioEngine';

const isCordova = !!window.cordova;
const readyEvent = isCordova ? 'deviceready' : 'load';
const RIOT_URL = isCordova ? 'ws://192.168.1.1' : 'ws://192.168.1.40';

const files = [
  // './assets/lux-aeterna-355Hz-10sec.wav',
  // './assets/sawtooth-497Hz-10sec.wav',
];

const audioContext = audio.audioContext;
const loader = new loaders.AudioBufferLoader();
const $container = document.querySelector('#app');

const app = {
  init(buffers) {
    this.buffers = buffers;

    this.main = this.main.bind(this);
    this.sync = this.sync.bind(this);

    this.view = new View($container);
    this.view.syncRequest = this.sync;
    this.view.render();

    this.motionInput = new lfoMotion.source.MotionInput();
  },

  sync() {
    // start audio for iOS
    const now = audioContext.currentTime;

    const gain = audioContext.createGain();
    gain.connect(audioContext.destination);
    gain.gain.value = 0;

    const osc = audioContext.createOscillator();
    osc.connect(gain);
    osc.start(now);
    osc.stop(now + 1);

    this.view.model.state = 'sync';
    this.view.render();

    const promises = [
      this.motionInput.init(),
      riot.open(RIOT_URL),
    ];

    // get riot and sensors
    Promise.all(promises)
      .then(this.main)
      .catch(err => console.error(err.stack));
  },

  main() {
    const now = audioContext.currentTime;

    // RioT mapping
    // --------------------------------------------------------------

    // feedback delay
    const feedback = audioContext.createGain();
    feedback.connect(audioContext.destination);
    feedback.gain.value = 0.92;

    const delay = audioContext.createDelay();
    delay.connect(feedback);
    feedback.connect(delay);
    delay.delayTime.value = 0.2;

    const preDelay = audioContext.createGain();
    preDelay.connect(feedback);
    preDelay.gain.value = 0;

    // lowpass
    const lowPass = audioContext.createBiquadFilter();
    lowPass.connect(preDelay);
    lowPass.connect(audioContext.destination);

    const minFreq = 200;
    const maxFreq = 6000;
    const freqRatio = Math.log2(maxFreq / minFreq);
    lowPass.frequency.value = minFreq;

    const scheduler = audio.getScheduler();
    const engine = new AudioEngine(lowPass);
    scheduler.add(engine);

    // Phone mapping
    // --------------------------------------------------------------

    const baseFreq = 300;
    // bourdon
    const sustained = audioContext.createGain();
    sustained.connect(audioContext.destination);
    sustained.gain.value = 0;


    const tremolo = audioContext.createGain();
    tremolo.connect(sustained);
    tremolo.gain.value = 0.7;

    const depth = audioContext.createGain();
    depth.connect(tremolo.gain);
    depth.gain.value = 0.3;

    const mod = audioContext.createOscillator();
    mod.connect(depth);
    mod.frequency.value = 1;
    mod.start(now);

    const sustainLowPass = audioContext.createBiquadFilter();
    sustainLowPass.frequency.value = 600;
    sustainLowPass.connect(tremolo);

    const osc0 = audioContext.createOscillator();
    osc0.connect(sustainLowPass);
    osc0.type = 'sawtooth';
    osc0.frequency.value = baseFreq;

    const osc1 = audioContext.createOscillator();
    osc1.connect(sustainLowPass);
    osc1.type = 'sawtooth';
    osc1.frequency.value = baseFreq * 3/2;

    const osc2 = audioContext.createOscillator();
    osc2.connect(sustainLowPass);
    osc2.type = 'sawtooth';
    osc2.frequency.value = baseFreq * Math.pow(2, 1/7);

    const osc3 = audioContext.createOscillator();
    osc3.connect(sustainLowPass);
    osc3.type = 'sawtooth';
    osc3.frequency.value = baseFreq * Math.pow(2, 1/7) * 3/2;

    osc0.start(now);
    osc1.start(now);
    osc2.start(now);
    osc3.start(now);

    // rendering
    this.view.model.state = 'main';
    this.view.render();

    // phone
    {
      let flag = false;

      const orientation = new lfoMotion.operator.Orientation();

      const select = new lfo.operator.Select({
        indexes: [2],
      });

      const bpfDisplay = new lfo.sink.BpfDisplay({
        canvas: '#phone-orientation',
        max: 0.5,
        min: -1.5,
        width: window.innerWidth,
        height: window.innerHeight / 2,
        radius: 4,
        line: false,
        duration: 5,
        colors: ['white'],
      });

      const bridge = new lfo.sink.Bridge({
        processFrame: frame => {
          const zAxis = frame.data[2];
          const sustainedGain = Math.max(0, Math.min(1, zAxis * -1));
          sustained.gain.value = Math.pow(sustainedGain, 3) * 0.13;

          const xAxis = frame.data[0];
          const depthFrequency = Math.max(1, Math.min(10, xAxis * -9 + 1));
          mod.frequency.value = depthFrequency;
        }
      });

      // connect graph
      this.motionInput.connect(orientation);

      orientation.connect(select)
      select.connect(bpfDisplay);

      orientation.connect(bridge);

      this.motionInput.start();
    }

    // riot
    {
      let flag = false;

      const eventIn = new lfo.source.EventIn({
        frameSize: 6,
        frameRate: 0,
        frameType: 'vector',
      });
      const orientation = new lfoMotion.operator.Orientation();

      const select = new lfo.operator.Select({
        indexes: [1],
      });

      const bpfDisplay = new lfo.sink.BpfDisplay({
        canvas: '#riot-orientation',
        min: -1,
        max: 1,
        width: window.innerWidth,
        height: window.innerHeight / 2,
        radius: 4,
        line: false,
        duration: 5,
        colors: ['#40de56'],
      });

      const bridge = new lfo.sink.Bridge({
        processFrame: frame => {
          const yAxis = frame.data[1];
          const ratio = Math.max(0, Math.min(1, yAxis + 1));
          const freq = minFreq * Math.pow(2, freqRatio * ratio);
          lowPass.frequency.value = freq;

          const zAxis = frame.data[2];
          const normValue = 1 - (zAxis + 1) * 0.5; // top 0 -> bottom 1
          const delayInGain = Math.min(1, Math.max(0, Math.sqrt(normValue) * 0.9));
          preDelay.gain.value = delayInGain;
          const eventProbability = normValue * 0.45 + 0.45;
          engine.eventProbability = eventProbability;
        },
      });

      eventIn.connect(orientation);

      orientation.connect(select);
      select.connect(bpfDisplay);

      orientation.connect(bridge);

      riot.setCallback(data => eventIn.process(null, data));
      eventIn.start();
    }
  },
};

window.addEventListener(readyEvent, () => {
  loader
    .load(files)
    .then(buffers => app.init(buffers));
}, false);
