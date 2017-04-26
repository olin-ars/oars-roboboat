'use strict'
// Connecting to ROS
// -----------------

var ros = setupROS('ws://' + prompt('ROS IP address', '192.168.17.130') + ':9090')

function setupROS(rosIP) {
    var ros = new ROSLIB.Ros({
        url : rosIP
    });

    ros.on('connection', function() {
        console.log('Connected to websocket server.');
        setup_subscribers(ros);
    });

    ros.on('error', function(error) {
        console.log('Error connecting to websocket server: ', error);
        window.setTimeout(setupROS, 5000);
    });

    ros.on('close', function() {
        console.log('Connection to websocket server closed.');
        window.setTimeout(setupROS, 5000);
    });

    return ros;
}

// Publishing a Topic
// ------------------

//   var cmdVel = new ROSLIB.Topic({
//     ros : ros,
//     name : '/html_test_output',
//     messageType : 'geometry_msgs/Twist'
//   });

//   var twist = new ROSLIB.Message({
//     linear : {
//       x : 0.1,
//       y : 0.2,
//       z : 0.3
//     },
//     angular : {
//       x : -0.1,
//       y : -0.2,
//       z : -0.3
//     }
//   });
//   cmdVel.publish(twist);

function setup_subscribers(ros) {
    
    // Location subscriber (handles local position)
    var listenloc = new ROSLIB.Topic({
        ros : ros,
        name : '/location',
        messageType : 'geometry_msgs/Pose2D'
    });

    listenloc.subscribe(function(message) {
        var string=message.x.toFixed(2) +",\t\t"+ message.y.toFixed(2) +",\t\t"+ message.theta.toFixed(2);
        //console.log('Received location : ' + string);



        var scale=50
        $("#local_pos").text(string);

        var tx = ((message.x*scale+250)%500+500)%500
        var ty = ((-message.y*scale+250)%500+500)%500
        $("#localboatmap").attr("transform","translate("+tx+","+ty+"), rotate("+message.theta+")")
    });



    // Sail subscriber(s)
    var listensail = new ROSLIB.Topic({
        ros : ros,
        name : '/sail/pos',
        messageType : 'std_msgs/Float32'
    });

    listensail.subscribe(function(message) {
        var string=message.data.toFixed(2);
        //console.log('Received Sail : ' + string);

        $("#sail_angle").text(string);

        var angle=message.data*12; //This should be multiplied for real boat values.
        $("#sailVis").attr("Transform","rotate("+angle+")")
    });



    // Rudder subscriber
    var listenrudder = new ROSLIB.Topic({
        ros : ros,
        name : '/rudder/pos',
        messageType : 'std_msgs/Int16'
    });

    listenrudder.subscribe(function(message) {
        var string=message.data.toFixed(2);
        //console.log('Received Rudder : ' + string);

        $("#rudder_angle").text(string);

        var angle = -message.data;
        $("#rudderVis").attr("Transform","rotate("+angle+")")
    });



    // Wind subscribers

        // True wind
    var listentruewind = new ROSLIB.Topic({
        ros : ros,
        name : '/true_wind',
        messageType : 'geometry_msgs/Pose2D'
    });

    listentruewind.subscribe(function(message) {
        var string=message.x.toFixed(2)+"@"+message.theta.toFixed(2);

        $("#true_wind").text(string);
    });  

        // Global wind
    var listenglobalwind = new ROSLIB.Topic({
        ros : ros,
        name : '/global_wind',
        messageType : 'geometry_msgs/Pose2D'
    });

    listenglobalwind.subscribe(function(message) {
        var string=message.x.toFixed(2)+"@"+message.theta.toFixed(2);
        //console.log('Received Rudder : ' + string);

        $("#global_wind").text(string);

         var angle=message.theta;
         $("#windVector1").attr("Transform","rotate("+angle+")")
    });


    // Autonomy internals
    // Note the use of an anonymous function to scope the topic variable locally
    (function() { 
        var topic = new ROSLIB.Topic({
            ros : ros,
            name : '/tacking',
            messageType : 'std_msgs/Bool'
        });

        topic.subscribe(function(message) {
            var string = ""+message.data;

            $("#is_tacking").text(string);
        });
    })();

    (function() { 
        var topic = new ROSLIB.Topic({
            ros : ros,
            name : '/going_upwind',
            messageType : 'std_msgs/Bool'
        });

        topic.subscribe(function(message) {
            var string = ""+message.data;

            $("#is_upwind").text(string);
        });
    })();

    (function() { 
        var topic = new ROSLIB.Topic({
            ros : ros,
            name : '/heading_err',
            messageType : 'std_msgs/Int16'
        });

        topic.subscribe(function(message) {
            var string = ""+message.data.toFixed(2);

            $("#heading_err").text(string);
        });
    })();

}



// Watchdog timers
var setup_watchdog = function(topic, msgtype, element, validfunc) {
    if (typeof validfunc == 'undefined') {
        validfunc = function() {return true;}
    }

    var lastMessage = 0;

    var sub = new ROSLIB.Topic({
        ros : ros,
        name : topic,
        messageType : msgtype
    });

    sub.subscribe(function(message) {
            if(validfunc(message))
                lastMessage = new Date().getTime()
        })

    var update = function() {
        var connected = (new Date().getTime() - lastMessage) < 2000;

        var bp = connected ? "" : "-64px 0px";

        element.css("background-position", bp);
    }

    window.setInterval(update, 500);
    update()
}

