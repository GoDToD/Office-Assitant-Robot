import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import speech_recognition as sr
import os
import sys
from openai import AzureOpenAI

class VoiceListener(Node):
    def __init__(self, test_audio_file=None):
        super().__init__('voice_listener')
        self.publisher_ = self.create_publisher(String, 'voice_command', 10)
        self.recognizer = sr.Recognizer()
        self.test_audio_file = test_audio_file

        # Setup OpenAI client (Azure)
        self.client = AzureOpenAI(
            api_key=os.environ["OPENAI_API_KEY"],
            api_version=os.environ["OPENAI_API_VERSION"],
            azure_endpoint=os.environ["OPENAI_API_BASE"]
        )
        self.model = os.environ.get("AZURE_OPENAI_DEPLOYMENT", "gpt-4")

        if self.test_audio_file:
            self.get_logger().info(f"🔊 Running in test mode with file: {self.test_audio_file}")
            self.run_test_file()
        else:
            self.microphone = sr.Microphone()
            self.get_logger().info("🎤 Running in microphone mode.")
            self.listen_loop()

    def run_test_file(self):
        try:
            with sr.AudioFile(self.test_audio_file) as source:
                audio = self.recognizer.record(source)
            text = self.recognizer.recognize_google(audio)
            self.get_logger().info(f"[TEST] Transcribed: {text}")
            intent = self.query_llm(text)
            self.publish_intent(intent)
        except Exception as e:
            self.get_logger().error(f"[TEST] Error processing audio file: {e}")

    def listen_loop(self):
        while rclpy.ok():
            try:
                with self.microphone as source:
                    self.get_logger().info("Listening...")
                    audio = self.recognizer.listen(source)
                text = self.recognizer.recognize_google(audio)
                self.get_logger().info(f"You said: {text}")
                intent = self.query_llm(text)
                self.publish_intent(intent)
            except Exception as e:
                self.get_logger().error(f"Error: {e}")

    def query_llm(self, text):
        prompt = f"""You are an intent parser for a robot assistant. Given the user's natural language instruction, return a JSON like:

{{
  "task": "pick",           // pick, go_to, deliver, etc.
  "object": "cola",         // optional, like 'cola'
  "location": null          // optional, like 'kitchen' or null
}}

Only return valid JSON. User said: "{text}"
"""

        response = self.client.chat.completions.create(
            model=self.model,
            messages=[{"role": "user", "content": prompt}],
            temperature=0.2
        )
        return response.choices[0].message.content.strip()

    def publish_intent(self, intent_json):
        msg = String()
        msg.data = intent_json
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published intent: {intent_json}")

def main(args=None):
    rclpy.init(args=args)
    test_file = None

    if len(sys.argv) > 1:
        test_file = sys.argv[1]

    node = VoiceListener(test_audio_file=test_file)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
