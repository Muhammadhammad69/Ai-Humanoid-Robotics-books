---
sidebar_position: 2
---

# باب 2: OpenAI Whisper کے ساتھ آواز سے ایکشن

## جائزہ

یہ باب ہیومنوڈ روبوٹس کے لیے آواز کی پروسیسنگ کی صلاحیتوں کو نافذ کرنے پر مرکوز ہے جس میں OpenAI Whisper کا استعمال کیا جاتا ہے، جو ایک جدید ترین اسپیچ ریکوگنیشن سسٹم ہے۔ آواز سے ایکشن پائپ لائن روبوٹس کو قدرتی زبان کے حکم کو سمجھنے اور انہیں ROS 2 فریم ورک کے اندر قابل انجام ایکشنز میں تبدیل کرنے کے قابل بناتی ہے۔ یہ صلاحیت جسمانی انسان-روبوٹ تعامل کو تخلیق کرنے کے لیے بنیادی ہے، جو صارفین کو روبوٹس کے ساتھ روزمرہ کی زبان کا استعمال کرتے ہوئے بات چیت کرنے کی اجازت دیتی ہے بجائے کہ مخصوص حکم کے۔

## سیکھنے کے اہداف

اس باب کے اختتام تک آپ درج ذیل کر سکیں گے:
- خودکار اسپیچ ریکوگنیشن (ASR) اور Whisper کے آرکیٹیکچر کے اصول سمجھیں
- روبوٹک ایپلی کیشنز کے لیے حقیقی وقت کی آواز کی پروسیسنگ پائپ لائنز نافذ کریں
- آواز کے حکم کی پروسیسنگ کے لیے Whisper کو ROS 2 کے ساتھ ضم کریں
- روبوٹک ایکشن منصوبہ بندی کے لیے ٹرانسکرائیبڈ اسپیچ سے منشاء نکالیں
- آواز کی پروسیسنگ میں عام چیلنجوں کو ہینڈل کریں جیسے کہ نوائز، ابیمبگویٹی، اور حقیقی وقت کی رکاوٹیں
- آواز کے تعاملات کے لیے ایرر ہینڈلنگ اور فیڈ بیک میکانزم نافذ کریں

## کلیدی تصورات

### خودکار اسپیچ ریکوگنیشن (ASR)
ASR وہ ٹیکنالوجی ہے جو بولی گئی زبان کو ٹیکسٹ میں تبدیل کرتی ہے۔ روبوٹک ایپلی کیشنز میں، ASR انسان کی آواز کے حکم اور روبوٹ ایکشن انجام دہی کے درمیان پل کا کام کرتا ہے۔ روبوٹک ASR کے لیے کلیدی غور شامل ہیں:
- حقیقی وقت کی پروسیسنگ کی ضروریات
- متحرک ماحول میں نوائز کی مزاحمت
- مختلف اکاؤسٹک حالات میں درستگی
- روبوٹک کنٹرول سسٹم کے ساتھ انضمام

### OpenAI Whisper آرکیٹیکچر
Whisper ایک ٹرانسفارمر-مبنی ماڈل ہے جو متعدد زبانوں اور اکاؤسٹک حالات میں اسپیچ ریکوگنیشن میں بہترین کارکردگی دکھاتا ہے۔ روبوٹکس کے لیے متعلقہ خصوصیات:
- متنوع صارف کے تعاملات کے لیے متعدد زبانوں کی حمایت
- نوائز اور آڈیو کی معیار کی تبدیلیوں کے خلاف مزاحمت
- قدرتی زبان کے حکم کے لیے بڑا لغت کا احاطہ
- پری-ٹرینڈ ماڈلز جنہیں مخصوص ڈومینز کے لیے فائن ٹیون کیا جا سکتا ہے

### حقیقی وقت کی آواز کی پروسیسنگ
روبوٹک ایپلی کیشنز کو قدرتی تعامل کے فلو کو برقرار رکھنے کے لیے کم لیٹنسی کے ساتھ حقیقی وقت کی آواز کی پروسیسنگ کی ضرورت ہوتی ہے:
- مسلسل سننے کے لیے اسٹریمنگ آڈیو پروسیسنگ
- بہترین کارکردگی کے لیے بفر مینجمنٹ
- جواب دہی کے تعامل کے لیے لیٹنسی کا خیال
- ایمبیڈڈ روبوٹک پلیٹ فارم کے لیے کمپیوٹیشنل کارآمدی

### منشاء کا انخلا
ٹرانسکرائیبڈ ٹیکسٹ کو قابل عمل حکم میں تبدیل کرنے کے لیے صارف کی منشاء کو سمجھنا ضروری ہے:
- ایکشن ایوربز کی شناخت کے لیے قدرتی زبان کی پروسیسنگ
- اشیاء، مقامات، اور پیرامیٹرز کے لیے اینٹیٹی ریکوگنیشن
- ابیمبگویٹی کے حل کے لیے متن کی آگاہی
- مخصوص روبوٹک صلاحیتوں کے ساتھ میپنگ

## تکنیکی گہرائی

### Whisper انضمام آرکیٹیکچر

ROS 2 کے ساتھ Whisper کا انضمام اس آرکیٹیکچر کو فالو کرتا ہے:

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   آڈیو ان پٹ   │───▶│   Whisper       │───▶│   منشاء         │
│   (مائیکروفون)  │    │   ٹرانسکرپشن  │    │   انخلا         │
│                 │    │                 │    │                 │
│ • خام آڈیو     │    │ • اسپیچ سے     │    │ • کمانڈ         │
│ • اسٹریمنگ     │    │   ٹیکسٹ        │    │   پارسنگ       │
│ • فارمیٹ        │    │ • متعدد        │    │ • اینٹیٹی       │
│   تبدیلی       │    │   زبانی         │    │   ریکوگنیشن   │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                              │
                              ▼
                    ┌─────────────────┐
                    │   ROS 2         │
                    │   انضمام        │
                    │                 │
                    │ • پیغام         │
                    │   شائع کرنا     │
                    │ • ایکشن        │
                    │   ٹریگر کرنا    │
                    └─────────────────┘
```

### آڈیو پروسیسنگ پائپ لائن

مکمل آڈیو پروسیسنگ پائپ لائن متعدد مراحل پر مشتمل ہے:

1. **آڈیو کی قبضہ**: مائیکروفونز یا دیگر آڈیو ذرائع سے خام آڈیو کو قبضہ کیا جاتا ہے
   - سیمپلنگ شرح کنفیگریشن (عام طور پر Whisper کے لیے 16kHz)
   - آڈیو فارمیٹ تبدیلی (WAV، FLAC، وغیرہ)
   - اسٹریمنگ پروسیسنگ کے لیے بفر مینجمنٹ

2. **پری پروسیسنگ**: Whisper پروسیسنگ کے لیے آڈیو کو تیار کیا جاتا ہے
   - نوائز کم کرنا اور فلٹرنگ
   - آڈیو نارملائزیشن
   - Whisper کی ضروریات کے لیے فارمیٹ تبدیلی

3. **ٹرانسکرپشن**: Whisper آڈیو کو ٹیکسٹ تیار کرنے کے لیے پروسیس کرتا ہے
   - پری-ٹرینڈ Whisper ماڈلز کا استعمال کرتے ہوئے ماڈل انفرینس
   - زبان کا پتہ لگانا اور ٹرانسکرپشن
   - ٹرانسکرپشن کی معیار کے لیے کمپلیمنس اسکورنگ

4. **پوسٹ-پروسیسنگ**: ٹرانسکرائیبڈ ٹیکسٹ کو منشاء کے انخلا کے لیے پروسیس کیا جاتا ہے
   - قدرتی زبان کی پروسیسنگ
   - منشاء کیسیفیکیشن
   - اینٹیٹی انخلا اور توثیق

### ROS 2 انضمام کے نمونے

Whisper معیاری میسجینگ نمونوں کا استعمال کرتے ہوئے ROS 2 کے ساتھ ضم ہوتا ہے:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from sensor_msgs.msg import AudioData
from openai import OpenAI
import whisper
import numpy as np
import threading
import queue

class WhisperVoiceProcessor(Node):
    """
    OpenAI Whisper کا استعمال کرتے ہوئے آواز کے حکم کو پروسیس کرنے کے لیے ROS 2 نوڈ۔
    آڈیو ان پٹ، ٹرانسکرپشن، اور منشاء کا انخلا ہینڈل کرتا ہے۔
    """

    def __init__(self):
        super().__init__('whisper_voice_processor')

        # Whisper ماڈل کو شروع کریں
        self.model = whisper.load_model("base")  # 'tiny', 'base', 'small', 'medium', 'large' ہو سکتا ہے

        # اضافی پروسیسنگ کے لیے OpenAI کلائنٹ کو شروع کریں اگر ضرورت ہو
        self.openai_client = OpenAI()  # اپنی API کلید کے ساتھ کنفیگر کریں

        # اسٹریمنگ پروسیسنگ کے لیے آڈیو بفر
        self.audio_buffer = queue.Queue()

        # پبلشرز
        self.transcript_publisher = self.create_publisher(
            String,
            'voice_transcript',
            10
        )

        self.command_publisher = self.create_publisher(
            String,
            'voice_command',
            10
        )

        self.feedback_publisher = self.create_publisher(
            String,
            'voice_feedback',
            10
        )

        # سبسکرائبرز
        self.audio_subscriber = self.create_subscription(
            AudioData,
            'audio_input',
            self.audio_callback,
            10
        )

        self.listening_control_subscriber = self.create_subscription(
            Bool,
            'voice_listening_control',
            self.listening_control_callback,
            10
        )

        # پیرامیٹرز
        self.declare_parameter('enable_voice_processing', True)
        self.declare_parameter('min_confidence_threshold', 0.7)
        self.declare_parameter('audio_buffer_size', 16000)  # 16kHz پر 1 سیکنڈ

        # اسٹیٹ متغیرات
        self.is_listening = True
        self.processing_thread = None
        self.shutdown_requested = False

        # آڈیو پروسیسنگ تھریڈ شروع کریں
        self.processing_thread = threading.Thread(target=self.process_audio_stream)
        self.processing_thread.start()

        self.get_logger().info('Whisper آواز پروسیسر شروع کیا گیا')

    def audio_callback(self, msg):
        """
        مائیکروفون سے آنے والے آڈیو ڈیٹا کے لیے کال بیک۔
        """
        if not self.is_listening or not self.get_parameter('enable_voice_processing').value:
            return

        # پروسیسنگ کی قطار میں آڈیو ڈیٹا شامل کریں
        audio_data = np.frombuffer(msg.data, dtype=np.int16).astype(np.float32) / 32768.0
        self.audio_buffer.put(audio_data)

        self.get_logger().debug(f'آڈیو چنک موصول ہوا: {len(audio_data)} نمونے')

    def listening_control_callback(self, msg):
        """
        کنٹرول کریں کہ آیا نوڈ آواز کے حکم کے لیے فعال طور پر سن رہا ہے۔
        """
        self.is_listening = msg.data
        self.get_logger().info(f'آواز سننا اس پر سیٹ ہوا: {self.is_listening}')

    def process_audio_stream(self):
        """
        Whisper کا استعمال کرتے ہوئے بفر سے مسلسل آڈیو پروسیس کریں۔
        """
        accumulated_audio = np.array([])

        while not self.shutdown_requested:
            try:
                # بفر سے آڈیو چنک حاصل کریں (ٹائم آؤٹ کے ساتھ)
                audio_chunk = self.audio_buffer.get(timeout=0.1)

                # پروسیسنگ کے لیے آڈیو جمع کریں
                accumulated_audio = np.concatenate([accumulated_audio, audio_chunk])

                # جب ہمارے پاس کافی آڈیو ہو (مثلاً 2 سیکنڈ کا)
                if len(accumulated_audio) >= self.get_parameter('audio_buffer_size').value * 2:
                    self.transcribe_and_process(accumulated_audio)
                    accumulated_audio = np.array([])  # بفر ری سیٹ کریں

            except queue.Empty:
                # کوئی آڈیو ڈیٹا دستیاب نہیں، لوپ جاری رکھیں
                continue
            except Exception as e:
                self.get_logger().error(f'آڈیو پروسیسنگ میں خامی: {e}')
                continue

    def transcribe_and_process(self, audio_data):
        """
        Whisper کا استعمال کرتے ہوئے آڈیو کو ٹرانسکرائیب کریں اور نتیجہ پروسیس کریں۔
        """
        try:
            # Whisper کی توقع کے مطابق فارمیٹ میں آڈیو تبدیل کریں
            # Whisper 1D float32 numpy ارے کے طور پر آڈیو کی توقع کرتا ہے
            if len(audio_data) > 0:
                # آڈیو کو ٹرانسکرائیب کریں
                result = self.model.transcribe(audio_data, language='en')

                transcript = result["text"].strip()
                confidence = result.get("avg_logprob", -1.0)  # کمپلیمنس کے طور پر لاگ ممکنہ کا استعمال کریں

                # خام ٹرانسکرپٹ شائع کریں
                transcript_msg = String()
                transcript_msg.data = transcript
                self.transcript_publisher.publish(transcript_msg)

                # کمپلیمنس کی حد چیک کریں
                min_conf = self.get_parameter('min_confidence_threshold').value
                if confidence > min_conf:
                    # حکم کو پروسیس کریں اور منشاء نکالیں
                    self.process_command(transcript)
                else:
                    self.get_logger().warn(f'کم کمپلیمنس ٹرانسکرپشن: {confidence}, ٹیکسٹ: {transcript}')
                    feedback_msg = String()
                    feedback_msg.data = f'حکم سمجھا نہیں گیا: کم کمپلیمنس ({confidence:.2f})'
                    self.feedback_publisher.publish(feedback_msg)

        except Exception as e:
            self.get_logger().error(f'ٹرانسکرپشن میں خامی: {e}')
            feedback_msg = String()
            feedback_msg.data = 'آواز کی پروسیسنگ میں خامی'
            self.feedback_publisher.publish(feedback_msg)

    def process_command(self, transcript):
        """
        ٹرانسکرائیبڈ حکم کو پروسیس کریں اور روبوٹ ایکشن کے لیے منشاء نکالیں۔
        """
        if not transcript:
            return

        self.get_logger().info(f'حکم کی پروسیسنگ: {transcript}')

        # سادہ منشاء انخلا - عمل میں، یہ زیادہ جامع NLP کا استعمال کرے گا
        command = self.extract_intent(transcript)

        if command:
            # پروسیسڈ حکم شائع کریں
            command_msg = String()
            command_msg.data = command
            self.command_publisher.publish(command_msg)

            # فیڈ بیک فراہم کریں
            feedback_msg = String()
            feedback_msg.data = f'حکم پہچانا گیا: {command}'
            self.feedback_publisher.publish(feedback_msg)
        else:
            feedback_msg = String()
            feedback_msg.data = f'حکم پہچانا نہیں گیا: {transcript}'
            self.feedback_publisher.publish(feedback_msg)

    def extract_intent(self, transcript):
        """
        ٹرانسکرائیبڈ ٹیکسٹ سے منشاء نکالیں۔
        یہ ایک سادہ ایمپلیمنٹیشن ہے - عمل میں، زیادہ جامع NLP کا استعمال کریں۔
        """
        transcript_lower = transcript.lower()

        # حکم کے پیٹرنز کی وضاحت کریں
        command_patterns = {
            'move_forward': ['move forward', 'go forward', 'move ahead', 'go ahead'],
            'move_backward': ['move backward', 'go backward', 'move back', 'go back'],
            'turn_left': ['turn left', 'rotate left', 'turn counter clockwise'],
            'turn_right': ['turn right', 'rotate right', 'turn clockwise'],
            'stop': ['stop', 'halt', 'pause'],
            'pick_up': ['pick up', 'grasp', 'grab', 'take'],
            'place': ['place', 'put', 'set down', 'release'],
            'follow': ['follow me', 'follow', 'come with me'],
            'find': ['find', 'locate', 'look for'],
            'bring': ['bring', 'get me', 'fetch']
        }

        # ٹرانسکرپٹ کو حکم کے پیٹرنز سے میچ کریں
        for command, patterns in command_patterns.items():
            for pattern in patterns:
                if pattern in transcript_lower:
                    return command

        # اگر کوئی مخصوص حکم نہیں ملا، اصل ٹرانسکرپٹ لوٹائیں
        return transcript

    def destroy_node(self):
        """
        نوڈ کو تباہ کرنے سے پہلے وسائل صاف کریں۔
        """
        self.shutdown_requested = True
        if self.processing_thread:
            self.processing_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    voice_processor = WhisperVoiceProcessor()

    try:
        rclpy.spin(voice_processor)
    except KeyboardInterrupt:
        pass
    finally:
        voice_processor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## کوڈ کی مثالیں

### Whisper انسٹالیشن اور سیٹ اپ

```bash
# آڈیو پروسیسنگ کے لیے Whisper انسٹال کریں
pip install openai-whisper

# اگر آپ کو torch/torchaudio کے ساتھ مسائل درپیش ہوں، مخصوص ورژن انسٹال کریں
pip install torch torchaudio --index-url https://download.pytorch.org/whl/cpu
pip install openai-whisper

# GPU ایکسلریشن کے لیے (اختیاری)
pip install torch torchaudio --index-url https://download.pytorch.org/whl/cu118
```

### روبوٹکس کے لیے اعلی درجے کا Whisper کنفیگریشن

```python
import whisper
import numpy as np
from dataclasses import dataclass
from typing import Optional

@dataclass
class WhisperConfig:
    """روبوٹک ایپلی کیشنز میں Whisper ماڈل کے لیے کنفیگریشن۔"""
    model_size: str = "base"  # tiny, base, small, medium, large
    language: str = "en"      # ریکوگنیشن کے لیے زبان
    task: str = "transcribe"  # transcribe یا translate
    beam_size: int = 5        # ڈیکوڈنگ کے لیے بیم سائز
    best_of: int = 5          # بہترین نتیجے کے لیے امیدواروں کی تعداد
    patience: float = 1.0     # بیم سرچ کے لیے برداشت
    temperature: float = 0.0  # نمونہ کے لیے درجہ حرارت
    compression_ratio_threshold: float = 2.4  # ناکامی کے پتہ لگانے کے لیے حد
    logprob_threshold: float = -1.0           # لاگ ممکنہ حد
    no_speech_threshold: float = 0.6          # خاموشی کے پتہ لگانے کے لیے حد

class RoboticWhisperProcessor:
    """روبوٹک ایپلی کیشنز کے لیے بہتر بنائے گئے Whisper پروسیسر کا اعلی درجہ۔"""

    def __init__(self, config: WhisperConfig = WhisperConfig()):
        self.config = config
        self.model = whisper.load_model(config.model_size)

        # آڈیو پروسیسنگ پیرامیٹرز
        self.sample_rate = 16000  # Whisper کے لیے معیار
        self.chunk_size = 1024   # اسٹریمنگ کے لیے آڈیو چنکس کا سائز

    def process_audio_chunk(self, audio_chunk: np.ndarray) -> Optional[str]:
        """
        آڈیو کا چنک پروسیس کریں اور اگر کمپلیمنس کافی ہو تو ٹرانسکرپشن لوٹائیں۔
        """
        # یقینی بنائیں کہ آڈیو صحیح فارمیٹ میں ہے
        if audio_chunk.dtype != np.float32:
            audio_chunk = audio_chunk.astype(np.float32)

        # آڈیو کو نارملائز کریں
        audio_chunk = audio_chunk / np.max(np.abs(audio_chunk)) if np.max(np.abs(audio_chunk)) != 0 else audio_chunk

        # مخصوص پیرامیٹرز کے ساتھ ٹرانسکرائیب کریں
        result = self.model.transcribe(
            audio_chunk,
            language=self.config.language,
            task=self.config.task,
            beam_size=self.config.beam_size,
            best_of=self.config.best_of,
            patience=self.config.patience,
            temperature=self.config.temperature,
            compression_ratio_threshold=self.config.compression_ratio_threshold,
            logprob_threshold=self.config.logprob_threshold,
            no_speech_threshold=self.config.no_speech_threshold
        )

        # کمپلیمنس چیک کریں اور نتیجہ لوٹائیں
        avg_logprob = result.get("avg_logprob", -float('inf'))
        if avg_logprob > self.config.logprob_threshold:
            return result["text"].strip()
        else:
            return None  # کم کمپلیمنس کا نتیجہ

    def process_streaming_audio(self, audio_generator):
        """
        حقیقی وقت میں اسٹریمنگ آڈیو کو پروسیس کریں۔
        """
        accumulated_audio = np.array([])

        for audio_chunk in audio_generator:
            accumulated_audio = np.concatenate([accumulated_audio, audio_chunk])

            # جب ہمارے پاس ٹرانسکرپشن کے لیے کافی آڈیو ہو
            if len(accumulated_audio) >= self.sample_rate * 2:  # 2 سیکنڈ کا آڈیو
                transcription = self.process_audio_chunk(accumulated_audio)
                if transcription:
                    yield transcription
                accumulated_audio = np.array([])  # ایکوومولیٹر ری سیٹ کریں
```

## ڈائریم (متن-مبنی)

### Whisper پروسیسنگ پائپ لائن

```
                    ┌─────────────────────────────────────────────────────────────┐
                    │                    آڈیو کی قبضہ                               │
                    │                (مائیکروفون ایرے)                            │
                    └─────────────────────────┬───────────────────────────────────┘
                                              │
                    ┌─────────────────────────▼───────────────────────────────────┐
                    │                  آڈیو پری پروسیسنگ                           │
                    │              (نوائز کم کرنا، فلٹرنگ)                        │
                    └─────────────────────────┬───────────────────────────────────┘
                                              │
                    ┌─────────────────────────▼───────────────────────────────────┐
                    │                   WHISPER ماڈل                              │
                    │                (اسپیچ ریکوگنیشن)                           │
                    └─────────────────────────┬───────────────────────────────────┘
                                              │
                    ┌─────────────────────────┼───────────────────────────────────┐
                    │                         │                                   │
        ┌───────────▼──────────┐    ┌────────▼────────┐    ┌─────────────────────▼─────────────┐
        │   ٹرانسکرپشن         │    │  کمپلیمنس       │    │      منشاء                       │
        │   (ٹیکسٹ آؤٹ پٹ)     │    │  چیکنگ         │    │  انخلا                          │
        │                      │    │                 │    │                                   │
        │ • خام ٹیکسٹ          │    │ • معیار         │    │ • حکم میپنگ                     │
        │ • زبان کا پتہ لگانا   │    │   اسکورنگ      │    │ • اینٹیٹی ریکوگنیشن            │
        │ • ٹائم اسٹیمپس       │    │ • حد            │    │ • متن کا انضمام                 │
        └──────────────────────┘    │   توثیق         │    └─────────────────────────────────┘
                                  └─────────────────┘
```

### ROS 2 انضمام آرکیٹیکچر

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   مائیکروفون    │───▶│ Whisper نوڈ     │───▶│ حکم منصوبہ بند │
│                 │    │                 │    │                 │
│ AudioData       │    │ voice_transcript│    │ voice_command   │
│ /audio_input    │    │ /voice_command  │    │ /voice_command  │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                              │
                              ▼
                    ┌─────────────────┐
                    │   ایکشن         │
                    │   ایگزیکیوٹر   │
                    │                 │
                    │ • نیوی گیشن    │
                    │ • مینیپولیشن   │
                    │ • فیڈ بیک      │
                    │ • سیفٹی        │
                    └─────────────────┘
```

## عام مسائل

### 1. آڈیو کی معیار کے مسائل
**مسئلہ**: خراب آڈیو کی معیار غلط ٹرانسکرپشن کا سبب بنتی ہے۔
**حل**: نوائز کم کرنا نافذ کریں، ڈائریکشنل مائیکروفونز کا استعمال کریں، اور آڈیو ان پٹ کی معیار کی توثیق کریں۔

### 2. حقیقی وقت کی پروسیسنگ لیٹنسی
**مسئلہ**: Whisper پروسیسنگ سست ہو سکتی ہے، جس کے نتیجے میں روبوٹ کے جواب میں تاخیر ہوتی ہے۔
**حل**: تیز پروسیسنگ کے لیے چھوٹے Whisper ماڈلز کا استعمال کریں، اسٹریمنگ نافذ کریں، اور کمپیوٹیشنل وسائل کو بہتر بنائیں۔

### 3. زبان کے ماڈل کا میس میچ
**مسئلہ**: جنرل آڈیو پر تربیت یافتہ Whisper روبوٹکس-مخصوص حکم پر اچھا کام نہیں کر سکتا۔
**حل**: روبوٹکس لغت پر Whisper کو فائن ٹیون کرنا غور کریں یا ڈومین-مخصوص زبان کے لیے پوسٹ-پروسیسنگ کا استعمال کریں۔

### 4. اسٹریمنگ میں متن کا نقصان
**مسئلہ**: آڈیو کو چنکس میں توڑنا سمجھنے کے لیے ضروری متن کے نقصان کا سبب بنتا ہے۔
**حل**: گفتگو کے متن کو برقرار رکھنے کے لیے اوور لیپنگ ونڈوز یا متن بفرز نافذ کریں۔

### 5. غلط ٹرگرنگ
**مسئلہ**: روبوٹ پس منظر کی گفتگو یا نوائز پر جواب دیتا ہے۔
**حل**: ویک ورڈ ڈیکشن، کمپلیمنس حدود، اور سلیکٹو سننے کے موڈ نافذ کریں۔

## چیک پوائنٹس

### سمجھ کا چیک 1: Whisper آرکیٹیکچر
- کیا آپ Whisper کو ٹیکسٹ تیار کرنے کے لیے آڈیو کو کیسے پروسیس کرتا ہے اس کی وضاحت کر سکتے ہیں؟
- مختلف Whisper ماڈل سائزز اور ان کے تنازعات کیا ہیں؟
- Whisper متعدد زبانوں کو کیسے ہینڈل کرتا ہے؟

### سمجھ کا چیک 2: ROS 2 انضمام
- Whisper نوڈ ROS 2 ایکوسسٹم میں کیسے فٹ ہوتا ہے؟
- آڈیو اور حکم کی ارسال کے لیے کون سے پیغام کی اقسام استعمال ہوتی ہیں؟
- آپ ROS 2 ٹاپکس کے ذریعے آواز کی پروسیسنگ سسٹم کو کیسے کنٹرول کر سکتے ہیں؟

### اطلاق کا چیک: آواز سے ایکشن پائپ لائن
- آپ اپنے مخصوص روبوٹک پلیٹ فارم کے لیے Whisper پروسیسر کو کیسے تبدیل کریں گے؟
- آپ غیر مناسب روبوٹ ایکشنز کو روکنے کے لیے کون سے سیفٹی اقدامات نافذ کریں گے؟
- آپ اپنی روبوٹک ایپلی کیشن میں مبہم آواز کے حکم کو کیسے ہینڈل کریں گے؟

## حوالہ جات

1. Radford, A., وغیرہ (2022). "بڑے پیمانے پر کمزور نگرانی کے ذریعے مضبوط اسپیچ ریکوگنیشن۔" *arXiv پری پرنٹ arXiv:2209.02364*۔

2. OpenAI. (2023). "Whisper: خودکار اسپیچ ریکوگنیشن۔" OpenAI دستاویزات۔ دستیاب: https://github.com/openai/whisper

3. ROS 2 دستاویزات. (2023). "ROS 2 پیغام کی اقسام اور کمیونیکیشن۔" دستیاب: https://docs.ros.org/en/humble/

4. Higuchi, T., وغیرہ (2023). "آڈیو ویژنل اسپیچ کی آن لائن ریکوگنیشن۔" *IEEE بین الاقوامی کانفرنس آن ایکوسٹکس، اسپیچ اور سگنل پروسیسنگ (ICASSP)*۔

5. NVIDIA. (2023). "Riva: GPU-ایکسلریٹڈ اسپیچ AI۔" NVIDIA ڈیولپر دستاویزات۔ دستیاب: https://developer.nvidia.com/nvidia-riva

6. Mozilla. (2023). "DeepSpeech: ایک TensorFlow ایمپلیمنٹیشن۔" دستیاب: https://github.com/mozilla/DeepSpeech

7. Hugging Face. (2023). "ٹرانسفارمرز: Pytorch، TensorFlow، اور JAX کے لیے جدید ترین مشین لرننگ۔" دستیاب: https://huggingface.co/docs/transformers

8. Picovoice. (2023). "PicoVoice Voice AI ٹیکنالوجیز۔" دستیاب: https://picovoice.ai/

9. SpeechRecognition لائبریری. (2023). "Python اسپیچ ریکوگنیشن لائبریری۔" دستیاب: https://pypi.org/project/SpeechRecognition/

10. CMU Sphinx. (2023). "Open Source اسپیچ ریکوگنیشن سسٹم۔" دستیاب: https://cmusphinx.github.io/