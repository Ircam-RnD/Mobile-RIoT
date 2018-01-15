let ws = null;
let timeoutId = null;

const riot = {
  ws: null,
  timeoutId: null,
  callback: null,
  buffer: new Float32Array(6),

  setCallback(callback) {
    this.callback = callback;
  },

  open(ip) {
    const promise = new Promise((resolve, reject) => {
      let counter = 0;

      const openSocket = () => {
        counter++;

        ws = new WebSocket(ip);
        ws.binaryType = 'blob';

        ws.addEventListener('open', e => {
          clearTimeout(timeoutId);
          timeoutId = null;

          resolve();
        });

        ws.addEventListener('message', e => {
          const buffer = this.buffer;
          const values = e.data.split(' ').map(val => parseFloat(val));
          // match phone sensors' order and units
          buffer[0] = values[3] / 100 * 9.81;
          buffer[1] = values[4] / 100 * 9.81;
          buffer[2] = values[5] / 100 * 9.81;
          buffer[3] = values[2];
          buffer[4] = values[0];
          buffer[5] = values[1];

          if (this.callback)
            this.callback(buffer);
        });

        ws.addEventListener('error', e => {
          if (counter >= 10)
            reject(new Error(`Fail to connect after ${counter} attempt`));
          else
            openSocket();
        });

        ws.addEventListener('close', e => console.log('close', e));

        console.log(`${counter} connection attempt`);
      }

      openSocket();
    });

    return promise;
  },
};

export default riot;
