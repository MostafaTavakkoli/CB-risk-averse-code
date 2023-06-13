def toJSON_Model(self, m, Bs):
    return_dict = {}

    for ct, idx in enumerate(Bs):
        for b in idx:
            if m[b]:
                return_dict[str(b)] = 'test'
            else:
                continue

    return {"edges": return_dict}


def toJSON_cGlo(self, m, cGlo):
    return_dict = {}

    for gL in cGlo:
        return_dict[str(gL)] = m[gL].as_long()

    return {"globalClocks": return_dict}


def toJSON_cLoc(self, m, cLoc):
    return_dict = {}

    for cL in cLoc:
        return_dict[str(cL)] = m[cL].as_long()

    return {"localClocks": return_dict}