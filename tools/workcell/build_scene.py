"""Source-checkout convenience entry; equivalent to rp-workcell."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'workflow'))
from r2s.workcell import main

if __name__ == '__main__':
    main()
