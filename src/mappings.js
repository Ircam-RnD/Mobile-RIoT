function getScale(domain, range) {
  const a = (range[1] - range[0]) / (domain[1] - domain[0]);
  const b = range[0] - domain[0] * a;

  const min = Math.min.apply(null, domain);
  const max = Math.max.apply(null, domain);

  return x => {
    x = Math.min(max, Math.max(min, x)); // clip
    return a * x + b;
  }
}

/*
mapping declaration could be

inputIndex: 0,
targetAttribute:
domain:
range: [0, 1200]
type: 'linear'

*/

export class PhoneMapping {
  constructor(bridge, synth) {
    this.synth = synth;

    this.ranges = [
      [0, 200],
      [0.01, 0.07],
      [0.1, 0.02],
      [1, 0],
    ];

    this.resamplingVarScale = getScale([1, 0], this.ranges[0]);
    this.periodScale = getScale([0, -1], this.ranges[1]);
    this.durationScale = getScale([0, -1], this.ranges[2]);
    this.volumeScale = getScale([0, 1], this.ranges[3]);

    this.processFrame = this.processFrame.bind(this);
    bridge.params.set('processFrame', this.processFrame);

    this.values = new Float32Array(4);
  }

  processFrame(frame) {
    const { data } = frame;
    const x = data[0];
    const y = data[1];
    const z = data[2];

    const resamplingVar = this.resamplingVarScale(y);
    this.synth.resamplingVar = resamplingVar;

    const period = this.periodScale(x);
    this.synth.periodAbs = period;

    const duration = this.durationScale(x);
    this.synth.durationAbs = duration;

    const volume = this.volumeScale(x);
    this.synth.gain = volume;

    // must be in same order as ranges
    this.values[0] = resamplingVar;
    this.values[1] = period;
    this.values[2] = duration;
    this.values[3] = volume;
  }
}

export class RiotMapping {
  constructor(bridge, synth) {
    this.synth = synth;

    this.ranges = [
      [0, 200],
      [1, 0.05],
      [0.01, 0.07],
      [0.1, 0.02],
    ];

    this.synth.position = this.synth.buffer.duration / 2;
    this.resamplingScale = getScale([-1, 0.8], this.ranges[0]);
    this.volumeScale = getScale([0, -1], this.ranges[1]);
    this.periodScale = getScale([0, -1], this.ranges[2]);
    this.durationScale = getScale([0, -1], this.ranges[3]);

    this.processFrame = this.processFrame.bind(this);
    bridge.params.set('processFrame', this.processFrame);

    this.values = new Float32Array(4);
  }

  processFrame(frame) {
    const { data } = frame;
    const x = data[0];
    const y = data[1];
    const z = data[2];

    const resampling = this.resamplingScale(z);
    this.synth.resampling = resampling;

    const volume = this.volumeScale(y);
    this.synth.gain = volume;

    const period = this.periodScale(x);
    this.synth.periodAbs = period;

    const duration = this.durationScale(x);
    this.synth.durationAbs = duration;

    this.values[0] = resampling;
    this.values[1] = volume;
    this.values[2] = period;
    this.values[3] = duration;
  }
}
