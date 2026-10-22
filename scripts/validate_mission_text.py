"""Validate exact UTF-8 mission bytes before model or motor initialization."""
import argparse
from pathlib import Path

def read_mission_text(filename):
    try:
        text=Path(filename).read_text(encoding='utf-8')
    except UnicodeDecodeError as exc:
        raise ValueError(f'Промпт повреждён: неверный UTF-8 около байта {exc.start}. Введите текст заново; содержимое не исправляется автоматически.') from exc
    except OSError as exc:
        raise ValueError(f'Не удалось прочитать промпт: {exc}') from exc
    if not text.strip() or len(text)>8000:
        raise ValueError('Промпт должен содержать от 1 до 8000 символов и не состоять из пробелов.')
    return text

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('filename')
    args=parser.parse_args()
    try:read_mission_text(args.filename)
    except ValueError as exc:
        print(str(exc))
        return 1
    return 0

if __name__=='__main__':raise SystemExit(main())
