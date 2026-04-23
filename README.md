# Grafik Tabanlı Termal SLAM Projesi

Graf optimizasyonu ve derin öğrenme tabanlı özellik çıkarımı kullanan, termal kamera verisiyle çalışan bir SLAM (Eşzamanlı Konumlandırma ve Haritalama) sistemi. 

## SLAM Nedir?
SLAM, bir robotun veya sistemin bilinmeyen bir ortamın haritasını oluştururken aynı anda kendi konumunu takip etmesini sağlar. Bu projede standart RGB kameralar yerine termal kamera görüntüleri kullanılmaktadır.

## Yaklaşım
- **Özellik Çıkarımı:** CNN tabanlı ve SSD tabanlı özellik çıkarıcıların karşılaştırılması  
- **Poz Tahmini:** CNN özellikleri kullanılarak kareler arası hareket (shift) tahmini  
- **Loop Closure:** Daha önce ziyaret edilen konumların tespiti ile sürüklenme (drift) düzeltmesi  
- **Graf Optimizasyonu:** Global tutarlılık için poz grafı oluşturma ve optimize etme  
- **Model:** Termal özellik çıkarımı için fine-tune edilmiş ResNet kullanımı  

## Kullanılan Teknolojiler
- MATLAB  
- CNN / ResNet (fine-tuned)  
- SSD (Single Shot Detector)  
- Poz Grafı Optimizasyonu  

## Dosya Yapısı
- `main.m` — Ana pipeline  
- `main_cnn.m` — CNN tabanlı pipeline  
- `main_ssd.m` — SSD tabanlı pipeline  
- `PoseGraph.m` — Graf oluşturma ve optimizasyon  
- `finetune_resnet.m` — ResNet fine-tuning  
- `evaluate_metrics.m` — Performans metrikleri  
- `results/` — Çıktı görselleştirmeleri  
