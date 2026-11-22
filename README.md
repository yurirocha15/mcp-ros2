# mcp-ros2

An [MCP](https://modelcontextprotocol.io) server for ROS 2. It lets an AI agent look at a running
ROS 2 system and act on it: list nodes, topics, services and actions, read and publish messages,
call services, send action goals, read and set parameters, and look up transforms.

The server is a ROS 2 node written in C++ on rclcpp and the
[MCP C++ SDK](https://github.com/yurirocha15/mcp-cpp-sdk). It needs nothing else running, such as
rosbridge, and it works with every installed message, service and action type: types are loaded by
name at run time, and messages go to and from JSON through their introspection layouts.

```text
### echo {"topic": "/turtle1/pose"}
{
  "messages": [
    {"angular_velocity": 0.0, "linear_velocity": 0.0, "theta": 0.0, "x": 5.5444446, "y": 5.5444446}
  ],
  "topic": "/turtle1/pose",
  "type": "turtlesim/msg/Pose"
}
### send_action_goal {"action": "/turtle1/rotate_absolute", "goal": {"theta": 0.0}}
{
  "accepted": true,
  "action": "/turtle1/rotate_absolute",
  "feedback_count": 105,
  "last_feedback": {"remaining": -0.0064},
  "result": {"delta": 1.664},
  "status": "succeeded",
  "type": "turtlesim/action/RotateAbsolute"
}
### publish {"topic": "/turtle1/cmd_vel", "message": {"linear": {"x": "fast"}}}  ERROR
linear.x: expected a number, not "fast"
```

These are calls an MCP client made to the server, with turtlesim running next to it.

## Tools

| Tool | What it does |
| --- | --- |
| `list_nodes` | The running nodes. |
| `list_topics` | Topics with their types and publisher and subscriber counts. |
| `list_services` | Services with their types, without every node's parameter services unless asked. |
| `list_actions` | Actions with their types. |
| `node_info` | What a node publishes, subscribes to, offers and uses. |
| `topic_info` | Who publishes and subscribes to a topic, with each one's QoS. |
| `describe_interface` | The fields of a message, service or action type. |
| `echo` | Waits for the next messages on a topic; `field` picks part of each one. |
| `topic_hz` | Message rate, the spread of the time between messages, and bandwidth. |
| `lookup_transform` | A transform from `/tf` and `/tf_static`, with roll, pitch and yaw. |
| `list_frames` | The coordinate frames heard of. |
| `get_parameters` | A node's parameters and their values. |
| `publish` | Publishes a message, once or several times at a rate. |
| `call_service` | Calls a service and returns its response. |
| `send_action_goal` | Sends a goal and waits for the result, reporting feedback as MCP progress. |
| `action_goal_status` | Follows a goal sent with `wait: false`: status, feedback, and the result. |
| `cancel_action_goal` | Cancels a goal sent with `wait: false`. |
| `set_parameters` | Sets a node's parameters. |

`publish`, `call_service`, `send_action_goal`, `cancel_action_goal` and `set_parameters` change the
running system and can move a robot. Their tool annotations mark them as destructive; `--read-only`
leaves them out, and `--allow` and `--deny` limit the names they may touch (see
[Safety](#safety)).

Long-running calls report progress, and a client can cancel them: a cancelled `echo` returns what
it received so far, and an action goal still running when its call is cancelled or times out is
cancelled on the action server. A goal that takes longer than a call should, such as a navigation
mission, can be sent with `wait: false`: the call returns a `goal_id` once the goal is accepted, the
goal runs on, and `action_goal_status` (which can wait for it) and `cancel_action_goal` take it from
there.

## Resources

- `ros://graph`: the nodes, topics, services and actions.
- `ros://topic/<name>`, such as `ros://topic/odom`: the newest message on a topic. A client that
  subscribes to it with `resources/subscribe` hears when new messages arrive, at most once a second.

## Messages as JSON

A message is a JSON object with the type's field names. Nested messages are objects and arrays are
JSON arrays; fields left out of a message you send keep their defaults. A
`builtin_interfaces/msg/Time` field, such as `header.stamp`, may be given as `"now"`. NaN and
infinity, which JSON lacks, are the strings `"nan"`, `"inf"` and `"-inf"`.

In messages the server returns, arrays longer than `--max-array` (64 by default; `echo` takes
`max_array` per call) come back as `{"length": n, "head": [the first values]}`, so a camera image
does not flood the agent's context.

When a message does not fit its type, the error names the field, as in
`linear.x: expected a number, not "fast"` or
`linear.q: geometry_msgs/msg/Vector3 has no such field; its fields are x, y, z`.

`echo` subscribes with a QoS that matches every publisher of the topic: reliable only when all of
them are, and transient-local only when all of them are. `topic_info` shows each side's QoS when
messages do not arrive.

## Requirements

ROS 2 Jazzy or Lyrical. The server uses the generic service and action clients rclcpp has had since
Jazzy, so Humble is not supported. CI builds it and runs its tests on Jazzy and Lyrical, on amd64 and
arm64, with Fast DDS, Cyclone DDS and Zenoh.

## Building

In a colcon workspace:

```sh
mkdir -p ~/mcp_ws/src && cd ~/mcp_ws/src
git clone https://github.com/yurirocha15/mcp-ros2.git
cd ~/mcp_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -y --skip-keys mcp_cpp_sdk
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

The MCP C++ SDK is not in the ROS package index yet; until it is, the build downloads it from
GitHub when it is not installed.

## Using it from an agent

The server speaks MCP over standard input and output, so the client starts it. It needs the ROS
environment of the system it joins, so source it first. For Claude Code:

```sh
claude mcp add ros2 -- bash -c \
  "source /opt/ros/jazzy/setup.bash && source ~/mcp_ws/install/setup.bash && exec ros2 run mcp_ros2 mcp_ros2"
```

For clients configured with JSON, such as Claude Desktop:

```json
{
  "mcpServers": {
    "ros2": {
      "command": "bash",
      "args": [
        "-c",
        "source /opt/ros/jazzy/setup.bash && source ~/mcp_ws/install/setup.bash && exec ros2 run mcp_ros2 mcp_ros2"
      ]
    }
  }
}
```

Set `ROS_DOMAIN_ID` in the command, as for any node, to join another domain. The server finds the
interface types of the packages in that environment, so source the workspace that holds your own
message packages too.

With `--transport=http` the server listens for MCP over HTTP instead, on 127.0.0.1:9797 by default.
Listening on another address needs a way to authenticate callers: a bearer token (`--token` or
`MCP_ROS2_TOKEN`), client certificates (`--tls-client-ca`), or both. `--tls-cert` and `--tls-key`
serve HTTPS, so the token does not cross the network in clear text:

```sh
ros2 run mcp_ros2 mcp_ros2 --transport=http --host=0.0.0.0 \
  --tls-cert=server.pem --tls-key=server-key.pem --token="$(cat token.txt)"
```

## Options

| Option | Default | Meaning |
| --- | --- | --- |
| `--read-only` | off | Leave out the tools that change the system, and `action_goal_status`. |
| `--allow=<pattern>` | any name | Let the tools that act touch only matching names; repeatable. |
| `--deny=<pattern>` | none | Never let them touch matching names; repeatable. |
| `--node-name=<name>` | `mcp_ros2` | The server's node name. |
| `--max-array=<n>` | 64 | Shorten longer arrays in returned messages. |
| `--discovery-wait=<s>` | 1.5 | Seconds after start that graph queries wait for discovery. |
| `--resource-wait=<s>` | 3 | Seconds a topic resource read waits for a message. |
| `--notify-interval=<s>` | 1 | Least seconds between notices of new messages on a subscribed topic. |
| `--transport=<stdio\|http>` | `stdio` | How clients connect. |
| `--host=<address>` | `127.0.0.1` | HTTP listen address. |
| `--port=<n>` | 9797 | HTTP listen port. |
| `--token=<token>` | `$MCP_ROS2_TOKEN` | Bearer token HTTP clients must send. |
| `--tls-cert=<file>` | none | Serve HTTPS with this PEM certificate chain. |
| `--tls-key=<file>` | none | The PEM private key of `--tls-cert`. |
| `--tls-client-ca=<file>` | none | Require client certificates from this PEM authority; repeatable. |

Discovery fills in the graph a moment after the server starts, so the first graph queries wait
`--discovery-wait` seconds from start; raise it for large graphs, slow networks or a remote Zenoh
router.

Arguments after `--ros-args` go to ROS, such as `--ros-args -r __ns:=/robot1` or
`--ros-args -p use_sim_time:=true`; with simulation time, `"now"` stamps follow `/clock`.

## Safety

An agent with this server can do what any node on the network can do: drive a robot, call any
service, and change parameters. Start with `--read-only` on real hardware, check the commands the
agent proposes, and keep the robot's own safety layers (velocity limits, emergency stops) in place.

`--allow` and `--deny` limit what the tools that act may touch: the topics `publish` publishes on,
the services `call_service` calls, the actions `send_action_goal` and `cancel_action_goal` use, and
the nodes `set_parameters` changes. Patterns are fully qualified ROS names, in which `*` matches
within one part of a name and `**` across parts. A name may be used when it matches an `--allow`
pattern, or none is given, and no `--deny` pattern. Reading is not limited. For example, to let an
agent drive one robot's base and nothing else:

```sh
ros2 run mcp_ros2 mcp_ros2 --allow=/robot1/cmd_vel --allow=/robot1/navigate_to_pose
```

The server's instructions tell the agent the rules, and a call outside them fails with a message
that names them.

## How it works

The server is one node, spun on a thread of its own. Each tool runs as a coroutine on the MCP side
and waits for the node's thread without blocking other calls. Types come from each interface
package's type support libraries, loaded by name; generic publishers, subscriptions, service clients
and action clients carry the messages, and `rosidl_typesupport_introspection_cpp` layouts convert
them to and from JSON.

## License

Apache-2.0
