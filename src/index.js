import * as lfo from 'waves-lfo/client';
import * as lfoMotion from 'lfo-motion';
import * as audio from 'waves-audio';
import * as loaders from 'waves-loaders';
import * as mappings from './mappings';
import riot from './riot';
import View from './View';
// import Renderer from './Renderer';
import AudioEngine from './AudioEngine';

const isCordova = !!window.cordova;
const readyEvent = isCordova ? 'deviceready' : 'load';
const RIOT_URL = isCordova ? 'ws://192.168.1.1' : 'ws://192.168.1.40';

const files = [
  './assets/lux-aeterna-355Hz-10sec.wav',
  './assets/sawtooth-497Hz-10sec.wav',
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
    preDelay.gain.setValueAtTime(0, now);

    // lowpass
    const lowPass = audioContext.createBiquadFilter();
    lowPass.connect(preDelay);
    lowPass.connect(audioContext.destination);

    const minFreq = 200;
    const maxFreq = 6000;
    const freqRatio = Math.log2(maxFreq / minFreq);
    lowPass.frequency.setValueAtTime(minFreq, now);

    const scheduler = audio.getScheduler();
    const engine = new AudioEngine(lowPass);
    scheduler.add(engine);


    //
    const sustained = audioContext.createGain();
    sustained.connect(audioContext.destination);
    sustained.gain.value = 0;

    const sustainedLowPass = audioContext.createBiquadFilter();
    sustainedLowPass.connect(sustained);
    sustainedLowPass.frequency.value = 1200;

    const osc0 = audioContext.createOscillator();
    osc0.connect(sustainedLowPass);
    osc0.frequency.value = 600;

    const osc1 = audioContext.createOscillator();
    osc1.connect(sustainedLowPass);
    osc1.frequency.value = 600 * 3/2;

    const osc2 = audioContext.createOscillator();
    osc2.connect(sustainedLowPass);
    osc2.frequency.value = 600 * Math.pow(2, 1/7);

    const osc3 = audioContext.createOscillator();
    osc3.connect(sustainedLowPass);
    osc3.frequency.value = 600 * Math.pow(2, 1/7) * 3/2;

    osc0.start(now);
    osc1.start(now);
    osc2.start(now);
    osc3.start(now);

    // rendering
    this.view.model.state = 'main';
    this.view.render();

    // phone
    {
      const orientation = new lfoMotion.operator.Orientation();
      const bridge = new lfo.sink.Bridge({
        processFrame: frame => {
          const data = frame.data;

          const zAxis = data[2];
          const sustainedGain = Math.max(0, Math.min(1, zAxis * -1));
          sustained.gain.value = Math.pow(sustainedGain, 3) * 0.05;
        }
      });

      const bpfDisplay = new lfo.sink.BpfDisplay({
        canvas: '#phone-orientation',
        min: -1,
        max: 1,
        width: window.innerWidth,
        height: window.innerHeight / 2,
      });

      // // connect graph
      this.motionInput.connect(orientation);
      orientation.connect(bridge);
      orientation.connect(bpfDisplay);

      this.motionInput.start();
    }

    // riot
    {
      const eventIn = new lfo.source.EventIn({
        frameSize: 6,
        frameRate: 0,
        frameType: 'vector',
      });
      const orientation = new lfoMotion.operator.Orientation();

      const bpfDisplay = new lfo.sink.BpfDisplay({
        canvas: '#riot-orientation',
        min: -1,
        max: 1,
        width: window.innerWidth,
        height: window.innerHeight / 2,
      });

      const bridge = new lfo.sink.Bridge({
        processFrame: frame => {
          // lowPass
          const now = audioContext.currentTime;
          const data = frame.data;

          const yAxis = data[1];
          const ratio = Math.max(0, Math.min(1, yAxis + 1));
          const freq = minFreq * Math.pow(2, freqRatio * ratio);
          lowPass.frequency.cancelScheduledValues(now);
          lowPass.frequency.setValueAtTime(lowPass.frequency.value, now);
          lowPass.frequency.linearRampToValueAtTime(freq, now + 0.02);

          const zAxis = data[2];
          const delayInGain = Math.min(1, Math.max(0, (1 - (zAxis + 1) * 0.5) * 0.6));
          preDelay.gain.cancelScheduledValues(now);
          preDelay.gain.setValueAtTime(preDelay.gain.value, now);
          preDelay.gain.linearRampToValueAtTime(delayInGain, now + 0.02);
        },
      });

      eventIn.connect(orientation);
      orientation.connect(bpfDisplay);
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
