import control
import matplotlib.pyplot as plt

# Transfer Fonksiyonu Parametreleri
K = 2
a = 1
b = 3
c = 70
d = 30

# Transfer Fonksiyonunu Oluşturma
numerator = [K, K*a]
denominator = [1, b+c, b*d+c, d*b]
transfer_function = control.TransferFunction(numerator, denominator)

# Zaman Cevabını Hesaplama
t, y = control.step_response(transfer_function)

# Grafiği Çizme
plt.figure(figsize=(10, 5))
plt.plot(t, y)
plt.title('3. Dereceden Transfer Fonksiyonunun Zaman Cevabı')
plt.xlabel('Zaman (s)')
plt.ylabel('Cevap')
plt.grid(True)
plt.show()