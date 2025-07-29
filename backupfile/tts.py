import speech_recognition as sr
from pydub import AudioSegment
from playsound import playsound
import openai
import tempfile
from gtts import gTTS
import tempfile, time, pygame
import time
import tempfile
openai.api_key = ''  # OpenAI API 키 입력
recognizer = sr.Recognizer()
def speak(text: str, speed: float = 1.3):
    print(":robeot_face::", text)
    # 1) TTS 생성
    with tempfile.NamedTemporaryFile(delete=False, suffix=".mp3") as fp:
        tts = gTTS(text=text, lang='ko')
        tts.save(fp.name)
        orig = fp.name
    # 2) pydub으로 속도 변경 (frame_rate 조작)
    sound = AudioSegment.from_file(orig)
    faster = sound._spawn(sound.raw_data, overrides={
        "frame_rate": int(sound.frame_rate * speed)
    }).set_frame_rate(sound.frame_rate)
    fast_path = orig.replace(".mp3", f"_fast_{int(speed*100)}.mp3")
    faster.export(fast_path, format="mp3")
    # 3) pygame으로 재생
    pygame.mixer.quit()
    pygame.mixer.init()
    pygame.mixer.music.load(fast_path)
    pygame.mixer.music.play()
    while pygame.mixer.music.get_busy():
        time.sleep(0.1)
def listen():
    with sr.Microphone() as source:
        print(":microphone: 말하세요...")
        audio = recognizer.listen(source)
        try:
            query = recognizer.recognize_google(audio, language='ko-KR')
            print(":bust_in_silhouette::", query)
            return query
        except sr.UnknownValueError:
            speak("무슨 말인지 잘 못 들었어요.")
        except sr.RequestError:
            speak("Google 음성 인식 서비스에 접근할 수 없습니다.")
        return None
def ask_chatgpt(prompt):
    response = openai.chat.completions.create(
        model="gpt-3.5-turbo",
        messages=[
            {"role": "system", "content": "너는 친절한 한국어 AI 비서야."},
            {"role": "user", "content": prompt}
        ]
    )
    return response.choices[0].message.content.strip()
while True:
    user_input = listen()
    if user_input:
        if "끝" in user_input or "종료" in user_input:
            speak("대화를 종료합니다. 좋은 하루 보내세요!")
            break
        reply = ask_chatgpt(user_input)
        speak(reply)
