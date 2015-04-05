// Connect to ROS bridge.
var ros = new ROSLIB.Ros({
  url: "ws://" + window.location.hostname + ":9090"
})

/**
 * Return ROS topics to subscribe to.
 */
function get_topics() {
  return [
    {
      obj: document.getElementById("battery"),
      name: "/battery",
      type: "std_msgs/String"
    }
  ]
}

/**
 * Subscribe to ROS topic.
 * @param {json} topic Topic to subscribe to.
 */
function subscribe(topic) {
  var listener = new ROSLIB.Topic({
    ros : ros,
    name : topic["name"],
    messageType : topic["type"]
  });
  listener.subscribe(function(message) {
    topic["obj"].innerHTML = message.data;
  });
}

/**
 * Subscribe to all given ROS topics.
 */
function subscribe_all(topics) {
  for (var i in topics) {
    var topic = topics[i];
    subscribe(topic);
  }
}
