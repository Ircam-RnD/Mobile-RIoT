import 'source-map-support/register';
import http from 'http';
import uws from 'uws';
import connect from 'connect';
import connectRoute from 'connect-route';
import path from 'path';
import portfinder from 'portfinder';
import serveStatic from 'serve-static';
import serveFavicon from 'serve-favicon';
import template from 'ejs-template';
// not very clean but...
import { getTranspiler } from '../../bin/runner';

const cwd = process.cwd();
const app = connect();

app.use(serveFavicon('./public/favicon.ico'));
app.use(serveStatic('./public'));
app.use(template.middleware({
  basedir: path.join(cwd, 'src', 'client'),
  autoreload: true,
}));

app.use(connectRoute(router => {
  const transpiler = getTranspiler();

  const serve = (name, req, res) => {
    // bundle the js file that correspond to the client
    const entryPoint = path.join(cwd, 'dist', 'client', `${name}.js`);
    const outFile = path.join(cwd, 'public', `${name}-bundle.js`);

    transpiler.bundle(entryPoint, outFile, () => {
      res.endTemplate('template.ejs', { name });
    });
  };

  router.get('/', (req, res, next) => serve('index', req, res));
  router.get('/:name', (req, res, next) => serve(req.params.name, req, res));
}));

const server = http.createServer(app);
const wss = new uws.Server({ server });

function sendData(socket, period) {
  (function send() {
    const data = [];
    for (let i = 0; i < 6; i++) {
      const rand = Math.random().toFixed(2);
      data.push(rand);
    }

    const msg = data.join(' ');
    socket.send(msg);
    // console.log('sent:', data);

    setTimeout(send, period);
  }());
}

wss.on('connection', (socket) => {
  sendData(socket, 20);

  socket.on('message', (arrayBuffer) => {
    console.log('received:', new Float32Array(arrayBuffer));
  });
});

server.listen(80, () => console.log(`Server started: http://127.0.0.1:80`));
