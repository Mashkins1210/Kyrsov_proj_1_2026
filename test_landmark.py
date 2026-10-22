# test_landmark.py
import cv2
import gc  # Добавляем сборщик мусора
from src.perception.landmark_detector import LandmarkDetector

def test_victim_detection():
    detector = LandmarkDetector()
    
    cap = cv2.VideoCapture(0)
    # Уменьшаем разрешение камеры для снижения нагрузки
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    frame_count = 0
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Детектируем жертву
        victims = detector._detect_victim(frame)
        
        # Рисуем результаты
        for victim in victims:
            x, y, w, h = victim['bbox']
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            label = f"Victim: {victim['confidence']:.2f}"
            cv2.putText(frame, label, (x, y - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            dist_label = f"Dist: {victim['distance']:.1f}m"
            cv2.putText(frame, dist_label, (x, y + h + 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        
        cv2.imshow('Victim Detection', frame)
        
        # Явный вызов сборщика мусора каждые 10 кадров
        frame_count += 1
        if frame_count % 10 == 0:
            gc.collect()
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    test_victim_detection()