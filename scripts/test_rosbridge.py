import websocket
import json
import time

def on_message(ws, message):
    data = json.loads(message)
    print(f"Received data: {data}")

def on_error(ws, error):
    print(f"Error: {error}")

def on_close(ws, close_status_code, close_msg):
    print("### closed ###")

def on_open(ws):
    # 订阅一个ROS话题，例如 /chatter
    subscribe_msg = {
        "op": "subscribe",
        "topic": "/chatter",
    }
    ws.send(json.dumps(subscribe_msg))
    
    # 向一个ROS话题发布消息，例如 /chatter
    publish_msg = {
        "op": "publish",
        "topic": "/chatter",
        "msg": {
            "data": "Hello, ROS!"
        }
    }
    
    # 延迟一会儿再发送消息，以确保连接已经稳定
    time.sleep(1)
    ws.send(json.dumps(publish_msg))

if __name__ == "__main__":
    websocket.enableTrace(True)
    ws = websocket.WebSocketApp("ws://localhost:9090/",
                                on_open=on_open,
                                on_message=on_message,
                                on_error=on_error,
                                on_close=on_close)

    ws.run_forever()
