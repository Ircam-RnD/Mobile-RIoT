import * as audio from 'waves-audio';

const audioContext = audio.audioContext;

function trigger(time, destination) {
  const rand = Math.random();
  const freqMult = Math.ceil(Math.random() * 9); // [1 .. 10]
  const jit = 0; // Math.random() * 20 * 0.001;
  const gain = (rand * rand) * 0.5 / freqMult;

  let baseFreq = 300;

  if (Math.random() < 0.5)
    baseFreq *= Math.pow(2, 1/7);

  const env = audioContext.createGain();
  env.connect(destination);

  const osc = audioContext.createOscillator();
  osc.connect(env);
  osc.frequency.value = baseFreq * freqMult;

  env.gain.value = 0;
  env.gain.setValueAtTime(0, time);
  env.gain.linearRampToValueAtTime(gain, time + 0.03);
  env.gain.exponentialRampToValueAtTime(0.0001, time + (1 + 2 / freqMult));

  osc.start(time + jit);
  osc.stop(time + 1);
}

class AudioEngine extends audio.TimeEngine {
  constructor(output) {
    super();

    this.output = output;
    this.period = 0.1;
  }

  advanceTime(time) {
    if (Math.random() < 0.8)
      trigger(time, this.output);

    return time + this.period;
  }

}

export default AudioEngine;
