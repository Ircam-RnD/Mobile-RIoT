import * as lfo from 'waves-lfo/client';
import * as lfoMotion from 'lfo-motion';
import * as audio from 'waves-audio';
import * as loaders from 'waves-loaders';
import * as mappings from './mappings';
import riot from './riot';
import View from './View';
import Renderer from './Renderer';

const RIOT_URL = 'ws://192.168.1.1';
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

    // bindings
    this.main = this.main.bind(this);
    this.sync = this.sync.bind(this);

    // init view
    this.view = new View($container);
    this.view.syncRequest = this.sync;
    this.view.render();

    //
    this.motionInput = new lfoMotion.source.MotionInput();
  },

  sync() {
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
    const scheduler = audio.getScheduler();
    // rendering
    this.view.model.state = 'main';
    this.view.render();

    const renderer = new Renderer(this.view.$canvas);

    this.master = audioContext.createGain();
    this.master.connect(audioContext.destination);
    this.master.gain.value = 0.8;

    // phone
    {
      const orientation = new lfoMotion.operator.Orientation();
      const bridge = new lfo.sink.Bridge();
      orientation.connect(bridge);

      const gain = audioContext.createGain();
      gain.connect(this.master);
      gain.gain.value = 0.6;

      const granularEngine = new audio.GranularEngine({
        buffer: this.buffers[0],
        position: this.buffers[0].duration / 2,
      });

      granularEngine.connect(gain);
      scheduler.add(granularEngine);

      const mapping = new mappings.PhoneMapping(bridge, granularEngine);

      this.motionInput.connect(orientation);
      orientation.connect(bridge);
      this.motionInput.start();

      renderer.add(mapping);
    }

    // riot
    {
      const eventIn = new lfo.source.EventIn({
        frameSize: 6,
        frameRate: 0,
        frameType: 'vector',
      });
      const orientation = new lfoMotion.operator.Orientation();
      const bridge = new lfo.sink.Bridge({
        processFrame: frame => console.log(frame.data),
      });

      const gain = audioContext.createGain();
      gain.connect(this.master);
      gain.gain.value = 0.4;

      const lowResonant = audioContext.createBiquadFilter();
      lowResonant.connect(gain);
      lowResonant.type = 'lowpass';
      lowResonant.frequency.value = 420 * 2;
      lowResonant.gain.value = 0.5;
      lowResonant.Q.value = 1.8;

      const lowpass = audioContext.createBiquadFilter();
      lowpass.connect(lowResonant);
      lowpass.type = 'lowpass';
      lowpass.frequency.value = 420 * 1;
      lowpass.gain.value = 0.8;
      lowpass.Q.value = 0.76;

      const bandpass = audioContext.createBiquadFilter();
      bandpass.connect(gain);
      bandpass.type = 'bandpass';
      bandpass.frequency.value = 420 * 3;
      bandpass.gain.value = 1;
      bandpass.Q.value = 20.2;

      const granularEngine = new audio.GranularEngine({
        buffer: this.buffers[1],
        position: this.buffers[1].duration / 2,
      });
      granularEngine.connect(lowpass);
      granularEngine.connect(bandpass);

      scheduler.add(granularEngine);

      const mapping = new mappings.RiotMapping(bridge, granularEngine);

      eventIn.connect(orientation);
      orientation.connect(bridge);
      riot.setCallback(data => eventIn.process(null, data));
      // riot.setCallback(data => console.log(data));

      eventIn.start();

      renderer.add(mapping);
    }

    renderer.start();
  },
};

document.addEventListener('deviceready', () => {
  loader
    .load(files)
    .then(buffers => app.init(buffers));
}, false);
