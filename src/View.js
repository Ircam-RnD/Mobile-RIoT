import template from 'lodash.template';

const tmpl = `
  <% if (state === 'init') { %>
    <button class="green-circle" id="sync">
      Sync
    </button>
  <% } else if (state === 'sync') { %>
    <button class="green-circle" disabled>
      Sync w/ R-IoT
    </button>
  <% } else if (state === 'main') { %>
    <canvas id="phone-orientation"></canvas>
    <canvas id="riot-orientation"></canvas>
  <% } %>
`;


class View {
  constructor($container) {
    this.model = {
      state: 'init',
    }

    this.$el = document.createElement('div');
    $container.appendChild(this.$el);

    this.tmpl = template(tmpl);
  }

  render() {
    const content = this.tmpl(this.model);
    this.$el.innerHTML = content;

    switch (this.model.state) {
      case 'init':
        const $btn = this.$el.querySelector('#sync');
        $btn.addEventListener('click', this.syncRequest);
        break;
      case 'main':
        // this.$canvas = this.$el.querySelector('#scene');
        // const width = window.innerWidth;
        // const height = window.innerHeight;
        // this.$canvas.width = width;
        // this.$canvas.height = height;
        break;
    }
  }
}

export default View;
