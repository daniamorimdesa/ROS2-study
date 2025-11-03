# ROS2 Bags: Gravação e Reprodução de Dados

## O que são Bags?
Bags são arquivos usados no ROS2 para gravar e reproduzir dados de tópicos, facilitando análise, debug e simulação.

## Criar pasta para armazenar bags (opcional)
```bash
mkdir bags
```

## Ver comandos disponíveis para bags
```bash
ros2 bag -h
```

## Gravar um tópico específico (exemplo)
```bash
ros2 bag record /number_count
```

## Gravar múltiplos tópicos
```bash
ros2 bag record /topico1 /topico2
```

## Listar arquivos de bag gravados
```bash
ls bags/
```

## Reproduzir uma bag
```bash
ros2 bag play <nome_da_bag>
```

## Dicas
- Use bags para registrar experimentos, simulações ou testes.
- Você pode filtrar tópicos e limitar o tempo de gravação.
- Consulte sempre `ros2 bag -h` para mais opções.
