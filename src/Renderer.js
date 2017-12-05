
class Renderer {
  constructor($canvas) {
    this.$canvas = $canvas;
    this.ctx = $canvas.getContext('2d');

    const boundingClientRect = $canvas.getBoundingClientRect();
    this.width = boundingClientRect.width;
    this.height = boundingClientRect.height;

    this.mappings = [];

    this.render = this.render.bind(this);
  }

  add(mapping) {
    this.mappings.push(mapping);
  }

  start() {
    this.dimensions = 0;
    this.mappings.forEach(mapping => this.dimensions += mapping.ranges.length);

    this.render();
  }

  render() {
    window.requestAnimationFrame(this.render);

    const ctx = this.ctx;
    const width = this.width;
    const height = this.height;

    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = '#40de56';

    let top = 0;
    const sliderHeight = height / this.dimensions;

    this.mappings.forEach(mapping => {
      mapping.ranges.forEach((range, index) => {
        const value = mapping.values[index];
        const min = Math.min.apply(null, range);
        const max = Math.max.apply(null, range);
        const norm = (value - min) / (max - min);

        ctx.fillRect(0, top, norm * width, sliderHeight);

        top += sliderHeight;
      });
    });
  }
}

export default Renderer;
