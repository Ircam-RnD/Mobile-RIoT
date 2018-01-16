(function e(t,n,r){function s(o,u){if(!n[o]){if(!t[o]){var a=typeof require=="function"&&require;if(!u&&a)return a(o,!0);if(i)return i(o,!0);var f=new Error("Cannot find module '"+o+"'");throw f.code="MODULE_NOT_FOUND",f}var l=n[o]={exports:{}};t[o][0].call(l.exports,function(e){var n=t[o][1][e];return s(n?n:e)},l,l.exports,e,t,n,r)}return n[o].exports}var i=typeof require=="function"&&require;for(var o=0;o<r.length;o++)s(r[o]);return s})({1:[function(require,module,exports){
'use strict';

function createSocket() {
  var url = window.location.origin.replace('http', 'ws');
  var socket = new WebSocket(url);
  socket.binaryType = 'arraybuffer';

  socket.addEventListener('open', function () {
    var data = new Float32Array([Math.random(), Math.random(), Math.random()]);
    socket.send(data.buffer);

    console.log('sent:', data);
  });

  socket.addEventListener('message', function (e) {
    console.log('received:', new Float32Array(e.data));
  });

  socket.addEventListener('error', function () {});
  socket.addEventListener('close', function () {});

  return socket;
}

function init() {
  var socket = createSocket();
  console.log(socket);
}

window.addEventListener('load', init);

},{}]},{},[1])
//# sourceMappingURL=data:application/json;charset=utf-8;base64,eyJ2ZXJzaW9uIjozLCJzb3VyY2VzIjpbIm5vZGVfbW9kdWxlcy9icm93c2VyLXBhY2svX3ByZWx1ZGUuanMiLCJkaXN0L2NsaWVudC9pbmRleC5qcyJdLCJuYW1lcyI6W10sIm1hcHBpbmdzIjoiQUFBQTs7O0FDQUEsU0FBUyxZQUFULEdBQXdCO0FBQ3RCLE1BQU0sTUFBTSxPQUFPLFFBQVAsQ0FBZ0IsTUFBaEIsQ0FBdUIsT0FBdkIsQ0FBK0IsTUFBL0IsRUFBdUMsSUFBdkMsQ0FBWjtBQUNBLE1BQU0sU0FBUyxJQUFJLFNBQUosQ0FBYyxHQUFkLENBQWY7QUFDQSxTQUFPLFVBQVAsR0FBb0IsYUFBcEI7O0FBRUEsU0FBTyxnQkFBUCxDQUF3QixNQUF4QixFQUFnQyxZQUFNO0FBQ3BDLFFBQU0sT0FBTyxJQUFJLFlBQUosQ0FBaUIsQ0FBQyxLQUFLLE1BQUwsRUFBRCxFQUFnQixLQUFLLE1BQUwsRUFBaEIsRUFBK0IsS0FBSyxNQUFMLEVBQS9CLENBQWpCLENBQWI7QUFDQSxXQUFPLElBQVAsQ0FBWSxLQUFLLE1BQWpCOztBQUVBLFlBQVEsR0FBUixDQUFZLE9BQVosRUFBcUIsSUFBckI7QUFDRCxHQUxEOztBQU9BLFNBQU8sZ0JBQVAsQ0FBd0IsU0FBeEIsRUFBbUMsVUFBQyxDQUFELEVBQU87QUFDeEMsWUFBUSxHQUFSLENBQVksV0FBWixFQUF5QixJQUFJLFlBQUosQ0FBaUIsRUFBRSxJQUFuQixDQUF6QjtBQUNELEdBRkQ7O0FBSUEsU0FBTyxnQkFBUCxDQUF3QixPQUF4QixFQUFpQyxZQUFNLENBQUUsQ0FBekM7QUFDQSxTQUFPLGdCQUFQLENBQXdCLE9BQXhCLEVBQWlDLFlBQU0sQ0FBRSxDQUF6Qzs7QUFFQSxTQUFPLE1BQVA7QUFDRDs7QUFFRCxTQUFTLElBQVQsR0FBZ0I7QUFDZCxNQUFNLFNBQVMsY0FBZjtBQUNBLFVBQVEsR0FBUixDQUFZLE1BQVo7QUFDRDs7QUFFRCxPQUFPLGdCQUFQLENBQXdCLE1BQXhCLEVBQWdDLElBQWhDIiwiZmlsZSI6ImdlbmVyYXRlZC5qcyIsInNvdXJjZVJvb3QiOiIiLCJzb3VyY2VzQ29udGVudCI6WyIoZnVuY3Rpb24gZSh0LG4scil7ZnVuY3Rpb24gcyhvLHUpe2lmKCFuW29dKXtpZighdFtvXSl7dmFyIGE9dHlwZW9mIHJlcXVpcmU9PVwiZnVuY3Rpb25cIiYmcmVxdWlyZTtpZighdSYmYSlyZXR1cm4gYShvLCEwKTtpZihpKXJldHVybiBpKG8sITApO3ZhciBmPW5ldyBFcnJvcihcIkNhbm5vdCBmaW5kIG1vZHVsZSAnXCIrbytcIidcIik7dGhyb3cgZi5jb2RlPVwiTU9EVUxFX05PVF9GT1VORFwiLGZ9dmFyIGw9bltvXT17ZXhwb3J0czp7fX07dFtvXVswXS5jYWxsKGwuZXhwb3J0cyxmdW5jdGlvbihlKXt2YXIgbj10W29dWzFdW2VdO3JldHVybiBzKG4/bjplKX0sbCxsLmV4cG9ydHMsZSx0LG4scil9cmV0dXJuIG5bb10uZXhwb3J0c312YXIgaT10eXBlb2YgcmVxdWlyZT09XCJmdW5jdGlvblwiJiZyZXF1aXJlO2Zvcih2YXIgbz0wO288ci5sZW5ndGg7bysrKXMocltvXSk7cmV0dXJuIHN9KSIsImZ1bmN0aW9uIGNyZWF0ZVNvY2tldCgpIHtcbiAgY29uc3QgdXJsID0gd2luZG93LmxvY2F0aW9uLm9yaWdpbi5yZXBsYWNlKCdodHRwJywgJ3dzJyk7XG4gIGNvbnN0IHNvY2tldCA9IG5ldyBXZWJTb2NrZXQodXJsKTtcbiAgc29ja2V0LmJpbmFyeVR5cGUgPSAnYXJyYXlidWZmZXInO1xuXG4gIHNvY2tldC5hZGRFdmVudExpc3RlbmVyKCdvcGVuJywgKCkgPT4ge1xuICAgIGNvbnN0IGRhdGEgPSBuZXcgRmxvYXQzMkFycmF5KFtNYXRoLnJhbmRvbSgpLCBNYXRoLnJhbmRvbSgpLCBNYXRoLnJhbmRvbSgpXSk7XG4gICAgc29ja2V0LnNlbmQoZGF0YS5idWZmZXIpO1xuXG4gICAgY29uc29sZS5sb2coJ3NlbnQ6JywgZGF0YSk7XG4gIH0pO1xuXG4gIHNvY2tldC5hZGRFdmVudExpc3RlbmVyKCdtZXNzYWdlJywgKGUpID0+IHtcbiAgICBjb25zb2xlLmxvZygncmVjZWl2ZWQ6JywgbmV3IEZsb2F0MzJBcnJheShlLmRhdGEpKTtcbiAgfSk7XG5cbiAgc29ja2V0LmFkZEV2ZW50TGlzdGVuZXIoJ2Vycm9yJywgKCkgPT4ge30pO1xuICBzb2NrZXQuYWRkRXZlbnRMaXN0ZW5lcignY2xvc2UnLCAoKSA9PiB7fSk7XG5cbiAgcmV0dXJuIHNvY2tldDtcbn1cblxuZnVuY3Rpb24gaW5pdCgpIHtcbiAgY29uc3Qgc29ja2V0ID0gY3JlYXRlU29ja2V0KCk7XG4gIGNvbnNvbGUubG9nKHNvY2tldCk7XG59XG5cbndpbmRvdy5hZGRFdmVudExpc3RlbmVyKCdsb2FkJywgaW5pdCk7XG4iXX0=
