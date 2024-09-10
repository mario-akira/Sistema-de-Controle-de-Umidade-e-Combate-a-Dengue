# Sistema-de-Controle-de-Umidade-e-Combate-a-Dengue

Durante um projeto de graduação, entre maio de 2024 e setembro de 2024, foi feito um protótipo de um sistema que controlasse a umidade de uma planta automaticamente e ainda retirasse o excesso de água para o cambate a dengue. 

![Imagem do WhatsApp de 2024-09-03 à(s) 11 25 40_e5269a9e](https://github.com/user-attachments/assets/80648465-ff03-41ce-862c-ecb4ca899efd)

Link de Vídeo para visualização de funcionamnto:https://youtu.be/r8OiV2AWL2Y

Foi utilizado um microcontrolador STM32F103C8T6(Bluepill) para controlar todo o sistema.

![Imagem do WhatsApp de 2024-09-10 à(s) 11 24 06_e1e05124](https://github.com/user-attachments/assets/7d7d0019-137d-4ef3-a254-153abed4f08a)

Foram utilizados dois sensores, um de umidade e outro de nível, ambos geram um sinal anlógico proporcional a variável física medida. Além disso, duas bombas pulverizadoras para irrigar e outra para retirar a água do prato quando o sensor medisse o risco de dengue.

![Imagem do WhatsApp de 2024-09-10 à(s) 11 24 07_75b35117](https://github.com/user-attachments/assets/5fbe0c6a-c397-410f-a5d7-66347f72aa55)

Além disso, com o uso de um display de LCD 2x16, foi criado um sistema de menu para as indicações do sistema e a determinação da umidade desejada para o sistema, sendo o menu controlado por três botões. Existem 5 janelas principais. Uma janela de avisos para indicar a situação do risco de dengue no sistema e de saúde do solo.

![Imagem do WhatsApp de 2024-09-10 à(s) 11 24 07_5b45f2bd](https://github.com/user-attachments/assets/78f24e11-45c2-496d-b9d4-7f9686cdd5d5)

Uma janela de verificação da umidade momentânea do solo e uma outra sub janela de determinação da umidade desejada para o controle.

![Imagem do WhatsApp de 2024-09-10 à(s) 11 24 07_c33e5455](https://github.com/user-attachments/assets/8cf47fbc-ee96-497d-904a-e187a3647e9a)

Uma janela de controle, ativado ou desativado, o controle automatico de umidade e combate a dengue

![Imagem do WhatsApp de 2024-09-10 à(s) 11 24 08_150eab88](https://github.com/user-attachments/assets/1639bfc9-1c42-4d85-a4ed-df415a4c2c90)

Uma janela para ativar ou desativar manualmente a irrigação da planta.

![Imagem do WhatsApp de 2024-09-10 à(s) 11 24 08_0bc08287](https://github.com/user-attachments/assets/e396cb6b-9c30-44ba-a3fb-aaf3b9c19c22)

E uma ultima janela para ativar ou desativar manualmente a retirada de água do prato.
