function createSocket() {
  const url = window.location.origin.replace('http', 'ws');
  const socket = new WebSocket(url);
  socket.binaryType = 'arraybuffer';

  socket.addEventListener('open', () => {
    const data = new Float32Array([Math.random(), Math.random(), Math.random()]);
    socket.send(data.buffer);

    console.log('sent:', data);
  });

  socket.addEventListener('message', (e) => {
    console.log('received:', new Float32Array(e.data));
  });

  socket.addEventListener('error', () => {});
  socket.addEventListener('close', () => {});

  return socket;
}

function init() {
  const socket = createSocket();
  console.log(socket);
}

window.addEventListener('load', init);
