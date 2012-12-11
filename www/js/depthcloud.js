(function (root, factory) {
    if (typeof define === 'function' && define.amd) {
        define(['three'], factory);
    } else {
        root.DepthCloud = factory(root.THREE);
    }
}
(
this,

function (THREE) {


    var DepthCloud = {};

    var Viewer = DepthCloud.Viewer = function (options) {

        // ///////////////////////////
        // depth cloud options
        // ///////////////////////////

        this.url = options.url;
        this.sceneNode = options.sceneNode;
        // f defaults to standard Kinect calibration
        this.f = (options.f !== undefined) ? options.f : 526;
        this.pointSize = (options.pointSize !== undefined) ? options.pointSize : 3;
        this.width = (options.width !== undefined) ? options.width : 1024;
        this.height = (options.height !== undefined) ? options.height : 1024;
        this.colorFader = (options.colorFader !== undefined) ? options.colorFader : 0;
        this.varianceFilter = (options.varianceFilter !== undefined) ? options.varianceFilter : 2950;

        var metaLoaded = false;
        this.video = document.createElement('video');

        this.video.addEventListener('loadedmetadata', this.metaLoaded.bind(this), false);

        this.video.loop = true;
        this.video.src = this.url;

        // ///////////////////////////
        // load shaders
        // ///////////////////////////

        var that = this;

        var vertex_shader, fragment_shader
        var shaderVXhr = new XMLHttpRequest();
        shaderVXhr.open("GET", "js/depthcloud_vs.shader", true);

        shaderVXhr.onload = function () {
            that.vertex_shader = shaderVXhr.responseText;
            that.initStreamer();
        };
        shaderVXhr.send(null);

        var shaderFXhr = new XMLHttpRequest();
        shaderFXhr.open("GET", "js/depthcloud_fs.shader", true);
        shaderFXhr.onload = function () {
            that.fragment_shader = shaderFXhr.responseText;
            that.initStreamer();
        };
        shaderFXhr.send(null);
    }

    Viewer.prototype.metaLoaded = function () {
        this.metaLoaded = true;
        this.initStreamer();
    }

    Viewer.prototype.initStreamer = function () {

        if ((this.vertex_shader != undefined) && (this.fragment_shader != undefined) && this.metaLoaded) {
            this.texture = new THREE.Texture(this.video);
            this.geometry = new THREE.Geometry();

            for (var i = 0, l = this.width * this.height; i < l; i++) {

                var vertex = new THREE.Vector3();
                vertex.x = (i % this.width);
                vertex.y = Math.floor(i / this.width);

                this.geometry.vertices.push(vertex);
            }

            this.material = new THREE.ShaderMaterial({

                uniforms: {

                    "map": {
                        type: "t",
                        value: this.texture
                    },
                    "width": {
                        type: "f",
                        value: this.width
                    },
                    "height": {
                        type: "f",
                        value: this.height
                    },
                    "focallength": {
                        type: "f",
                        value: this.f
                    },
                    "pointSize": {
                        type: "f",
                        value: this.pointSize
                    },
                    "zOffset": {
                        type: "f",
                        value: 1000
                    },
                    "colorFader": {
                        type: "f",
                        value: this.colorFader
                    },
                    "varianceFilter": {
                        type: "f",
                        value: this.varianceFilter
                    }
                },
                vertexShader: this.vertex_shader,
                fragmentShader: this.fragment_shader
                // depthWrite: false

            });

            this.mesh = new THREE.ParticleSystem(this.geometry, this.material);
            this.mesh.position.x = 0;
            this.mesh.position.y = 0;
            this.sceneNode.add(this.mesh);

            var that = this;
            setInterval(

            function () {

                if (that.video.readyState === that.video.HAVE_ENOUGH_DATA) {
                    that.texture.needsUpdate = true;
                }

            }, 1000 / 30);
        }


    }

    Viewer.prototype.startStream = function () {
        this.video.play();
    }

    Viewer.prototype.stopStream = function () {
        this.video.stop();
    }

    return DepthCloud;
}));