// Only the transport is adapted; the production JS session and codecs run unchanged.
import assert from 'node:assert/strict';
import net from 'node:net';
import path from 'node:path';
import { pathToFileURL } from 'node:url';

const sdkPath = path.resolve(process.argv[2], 'src/commands/kinisi_session.js');
const { KinisiSession, ControllerError, Topic } = await import(pathToFileURL(sdkPath));

class TCPClient extends KinisiSession {
  constructor(options) {
    super(options);
    this.queue = [];
    this.waiter = null;
  }
  async connect(port) {
    this.socket = net.createConnection({ host: '127.0.0.1', port });
    this.socket.setNoDelay(true);
    this.socket.on('data', data => {
      if (this.waiter) { const resolve = this.waiter; this.waiter = null; resolve(data); }
      else this.queue.push(data);
    });
    this.socket.on('close', () => { if (this.waiter) { this.waiter(new Uint8Array()); this.waiter = null; } });
    await new Promise((resolve, reject) => {
      this.socket.once('connect', resolve);
      this.socket.once('error', reject);
    });
    await this._startSession();
  }
  _readBytes() {
    if (this.queue.length) return Promise.resolve(this.queue.shift());
    return new Promise(resolve => { this.waiter = resolve; });
  }
  _writeBytes(data) {
    return new Promise((resolve, reject) => this.socket.write(data, error => error ? reject(error) : resolve()));
  }
  async disconnect() {
    this._endSession();
    this.socket?.destroy();
    if (this.waiter) { this.waiter(new Uint8Array()); this.waiter = null; }
    await this._readerTask;
  }
}

const client = new TCPClient({ wallClock: true, requestTimeoutMs: 1000 });
try {
  await client.connect(Number(process.argv[3]));
  assert.equal(client.ready, true);
  assert.equal(client.clockMode, 1);
  await assert.rejects(client.set_motor_target_speed(0, 1), error => error instanceof ControllerError && error.code === 14);
  await client.initialize_motor_controller(0, false, 0, false, 1000, 1, 0, 0, 100);
  await client.set_motor_target_speed(0, 2);
  await new Promise(resolve => setTimeout(resolve, 120)); // Production controller task period is 100 ms.
  const state = await client.get_motor_controller_state(0);
  assert.equal(state.target_speed, 2);
  await client.initialize_gpio_pin(1, 3);
  await client.set_gpio_pin_state(1, 1);
  assert.equal(await client.get_gpio_pin_state(1), 1);
  let resolveSample;
  const sample = new Promise(resolve => { resolveSample = resolve; });
  const subscription = await client.subscribe({ topic: Topic.TEMPERATURE, index: 0, intervalMs: 250 }, resolveSample);
  let timer;
  try {
    const value = await Promise.race([sample, new Promise((_, reject) => { timer = setTimeout(() => reject(new Error('No telemetry')), 2000); })]);
    assert.equal(value.celsius, 25);
    assert.equal(value.clock_mode, 1);
  } finally { clearTimeout(timer); }
  await subscription.unsubscribe();
  await client.stop_motor(0);
  await client.ping();
  console.log('JavaScript SDK integration passed: handshake, errors, motor, GPIO, telemetry, unsubscribe');
} finally {
  await client.disconnect();
}
