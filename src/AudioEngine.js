import * as audio from 'waves-audio';

const audioContext = audio.audioContext;
const pitchRatio = Math.pow(2, 1/7);

function trigger(time, destination) {
  const rand = Math.random();
  const partialIndex = Math.ceil(Math.random() * 9); // [1 .. 10]
  const jit = 0;
  const gain = (rand * rand) * 0.3 / partialIndex;
  const duration = (1 + 2 / partialIndex);

  let baseFreq = 300;

  if (Math.random() < 0.5)
    baseFreq *= pitchRatio;

  const env = audioContext.createGain();
  env.connect(destination);

  const osc = audioContext.createOscillator();
  osc.connect(env);
  osc.frequency.value = baseFreq * partialIndex;

  time = Math.max(audioContext.currentTime, time);

  env.gain.value = 0;
  env.gain.setValueAtTime(0, time);
  env.gain.linearRampToValueAtTime(gain, time + 0.03);
  env.gain.exponentialRampToValueAtTime(0.0001, time + duration);

  osc.start(time);
  osc.stop(time + duration);
  osc.onend = () => osc.disconnect();
}

class Ostinato {
  constructor(output) {
    this.baseFreq = 150;
    this.index = 0;
    this.gain = 0;
    this.output = output;
  }

  trigger(time, gain) {
    let freq = this.baseFreq;

    if (this.index % 2 === 1)
      freq *= pitchRatio;

    const partialIndex = Math.floor(this.index / 2) + 1;
    const duration = (1 + 2 / partialIndex);

    const env = audioContext.createGain();
    env.connect(this.output);
    env.gain.value = 0;
    env.gain.setValueAtTime(0, time);
    env.gain.linearRampToValueAtTime(gain, time + 0.03);
    env.gain.exponentialRampToValueAtTime(0.0001, time + duration);

    const osc = audioContext.createOscillator();
    osc.connect(env);
    osc.type = 'sawtooth';
    osc.frequency.value = freq *= partialIndex;
    osc.start(time);
    osc.stop(time + duration);
  }
}

class AudioEngine extends audio.TimeEngine {
  constructor(output) {
    super();

    this.eventProbability = 0;
    this.output = output;
    this.period = 0.075;

    this.ostinato = new Ostinato(audioContext.destination);
    this.ostinatoGain = 0;
  }

  advanceTime(time) {
    if (Math.random() < this.eventProbability)
      trigger(time, this.output);

    if (this.ostinatoGain > 0.001)
      this.ostinato.trigger(time, this.ostinatoGain);

    this.ostinato.index += 1;

    if (this.ostinato.index === 12)
      this.ostinato.index = 0;

    return time + this.period;
  }

}

export default AudioEngine;
