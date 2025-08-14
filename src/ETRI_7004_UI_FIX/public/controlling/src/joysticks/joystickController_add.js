class JoystickController {
    constructor(ros, cmdVel, onvifCamera) {
        this.ros = ros;
        this.cmdVel = cmdVel;
        this.onvifCamera = onvifCamera;
        this.maxLinearSpeed = 1/2;
        this.maxAngularSpeed = 3.14/4;
        this.preCameraMove = { pan: 0, tilt: 0 };
        this.socket = io();
        this.init();
        this.connectToCamera();
        this.lastSentTime = 0;
        this.messageInterval = 100;
        this.totalZoom = 0;

        this.heightMin = -0.05; this.heightMax = 0.05; this.heightStep = 0.005; this.heightCmd = 0.0;
        this.gripMin   = -0.06; this.gripMax   = 0.04; this.gripStep   = 0.0025; this.gripCmd   = 0.0;

        this.heightPub = new ROSLIB.Topic({ ros:this.ros, name:'/gripper_height_controller/commands', messageType:'std_msgs/Float64MultiArray' });
        this.gripPub   = new ROSLIB.Topic({ ros:this.ros, name:'/gripper_controller/commands',        messageType:'std_msgs/Float64MultiArray' });

        this.jointStates = new ROSLIB.Topic({ ros:this.ros, name:'/joint_states', messageType:'sensor_msgs/JointState' });
        this.jointStates.subscribe((msg) => {
            const hi = msg.name.indexOf('connecter_to_arm');
            if (hi >= 0) this.heightCmd = msg.position[hi];
            const li = msg.name.indexOf('left_finger_joint');
            if (li >= 0) this.gripCmd = msg.position[li];
        });

        this.initPTZButtons();
    }

    connectToCamera() {
        this.socket.emit('connection');
        if (this.onvifCamera) this.onvifCamera.connect(() => {});
    }

    init() {
        window.addEventListener("gamepadconnected", () => {
            this.connectToCamera();
            requestAnimationFrame(this.updateGamepadStatus.bind(this));
        });
        window.addEventListener("gamepaddisconnected", () => {});
    }

    publishHeight() {
        const v = Math.max(this.heightMin, Math.min(this.heightMax, this.heightCmd));
        this.heightCmd = v;
        this.heightPub.publish(new ROSLIB.Message({ layout:{dim:[],data_offset:0}, data:[v] }));
    }

    publishGrip() {
        const v = Math.max(this.gripMin, Math.min(this.gripMax, this.gripCmd));
        this.gripCmd = v;
        this.gripPub.publish(new ROSLIB.Message({ layout:{dim:[],data_offset:0}, data:[v, v] }));
    }

    updateGamepadStatus() {
        const pads = navigator.getGamepads && navigator.getGamepads();
        const gamepad = pads && pads[0];
        if (!gamepad) { requestAnimationFrame(this.updateGamepadStatus.bind(this)); return; }

        const now = Date.now();
        if (now - this.lastSentTime < this.messageInterval) { requestAnimationFrame(this.updateGamepadStatus.bind(this)); return; }

        if (gamepad.buttons[3]?.pressed) this.zoomControll("zoomIn");
        if (gamepad.buttons[1]?.pressed) this.zoomControll("zoomOut");

        if (gamepad.buttons[12]?.pressed) { this.heightCmd += this.heightStep; this.publishHeight(); }
        if (gamepad.buttons[13]?.pressed) { this.heightCmd -= this.heightStep; this.publishHeight(); }

        if (gamepad.buttons[15]?.pressed) { this.gripCmd += this.gripStep; this.publishGrip(); } // open
        if (gamepad.buttons[14]?.pressed) { this.gripCmd -= this.gripStep; this.publishGrip(); } // close

        const vx = parseFloat(gamepad.axes[0] * this.maxAngularSpeed);
        const vy = parseFloat(-gamepad.axes[1] * this.maxLinearSpeed);

        let twist;
        if (gamepad.buttons[4]?.pressed) twist = new ROSLIB.Message({ linear: { x: vy, y: -vx, z: 0 }, angular: { x: 0, y: 0, z: 0 } });
        else if (gamepad.buttons[6]?.pressed) twist = new ROSLIB.Message({ linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } });
        else twist = new ROSLIB.Message({ linear: { x: vy, y: 0, z: 0 }, angular: { x: 0, y: 0, z: -vx } });

        new ROSLIB.Topic({ ros:this.ros, name:this.cmdVel, messageType:"geometry_msgs/Twist" }).publish(twist);

        if (gamepad.buttons[5]?.pressed) this.zeroPointCamera();

        const rx = gamepad.axes[2], ry = gamepad.axes[3];
        const mag = Math.sqrt(rx*rx + ry*ry), speed = mag > 1 ? 1 : mag;
        if (speed > 0.1) {
            let pan = 0, tilt = 0;
            if (Math.abs(rx) > Math.abs(ry)) pan = rx * speed; else tilt = -ry * speed;
            this.moveCamera({ x: pan/9, y: tilt/9, zoom: 0 });
        }

        this.lastSentTime = now;
        requestAnimationFrame(this.updateGamepadStatus.bind(this));
    }

    initPTZButtons() {
        const up = document.getElementById('ptz-up');
        const down = document.getElementById('ptz-down');
        const left = document.getElementById('ptz-left');
        const right = document.getElementById('ptz-right');
        const zin = document.getElementById('ptz-zoom-in');
        const zout = document.getElementById('ptz-zoom-out');
        const home = document.getElementById('ptz-home');
        if (up)   up.onclick   = () => { this.heightCmd += this.heightStep; this.publishHeight(); };
        if (down) down.onclick = () => { this.heightCmd -= this.heightStep; this.publishHeight(); };
        if (left) left.onclick = () => { this.gripCmd  -= this.gripStep;  this.publishGrip();   };
        if (right)right.onclick= () => { this.gripCmd  += this.gripStep;  this.publishGrip();   };
        if (zin)  zin.onclick  = () => this.moveCamera({ x:0, y:0, zoom:0.1 });
        if (zout) zout.onclick = () => this.moveCamera({ x:0, y:0, zoom:-0.1 });
        if (home) home.onclick = () => this.zeroPointCamera();
    }

    moveCamera(m) { this.socket.emit('moveCamera', m); }
    zeroPointCamera() { this.socket.emit('zeroPointCamera'); }
    zoomControll(d) { const s=0.1; this.totalZoom+= d==="zoomIn"? s:-s; this.moveCamera({x:0,y:0,zoom:d==="zoomIn"? s:-s}); }
}

