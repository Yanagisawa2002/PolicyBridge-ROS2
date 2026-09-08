# Portfolio figure

This sequence illustrates the implemented cancellation/late-result boundary. It is an explanatory diagram, not a measured timing trace.

## Reproduce

From the repository root:

```bash
python -m pip install -r docs/portfolio/requirements.txt
python docs/portfolio/render.py
```

The renderer verifies source SHA-256 hashes (CRLF normalized to LF) before plotting the reviewed values in `figure.json`. If a source changes, review and refresh the snapshot before regenerating. It writes SVG and PNG with matching content.

## Sources

- [policy_bridge/policy_bridge/policy_server.py](../../policy_bridge/policy_bridge/policy_server.py)
- [policy_bridge/policy_bridge/runtime_state.py](../../policy_bridge/policy_bridge/runtime_state.py)
- [docs/validation.md](../../docs/validation.md)

The flow/memory/timing illustrations are schematics. Only explicitly labeled measurements represent recorded experiments. Confidence intervals are copied from source reports, not recomputed.
