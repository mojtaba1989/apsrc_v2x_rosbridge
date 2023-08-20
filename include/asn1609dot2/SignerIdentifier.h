/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "IEEE1609dot2"
 * 	found in "../IEEE-1609.2-2016.asn1"
 * 	`asn1c -S ../../skeletons -pdu=Certificate -pdu=auto -pdu=Ieee1609Dot2Data -fcompound-names`
 */

#ifndef	_SignerIdentifier_H_
#define	_SignerIdentifier_H_


#include <asn_application.h>

/* Including external dependencies */
#include "HashedId8.h"
#include "SequenceOfCertificate.h"
#include <NULL.h>
#include <constr_CHOICE.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dependencies */
typedef enum SignerIdentifier_PR {
	SignerIdentifier_PR_NOTHING,	/* No components present */
	SignerIdentifier_PR_digest,
	SignerIdentifier_PR_certificate,
	SignerIdentifier_PR_self
	/* Extensions may appear below */
	
} SignerIdentifier_PR;

/* SignerIdentifier */
typedef struct SignerIdentifier {
	SignerIdentifier_PR present;
	union SignerIdentifier_u {
		HashedId8_t	 digest;
		SequenceOfCertificate_t	 certificate;
		NULL_t	 self;
		/*
		 * This type is extensible,
		 * possible extensions are below.
		 */
	} choice;
	
	/* Context for parsing across buffer boundaries */
	asn_struct_ctx_t _asn_ctx;
} SignerIdentifier_t;

/* Implementation */
extern asn_TYPE_descriptor_t asn_DEF_SignerIdentifier;
extern asn_CHOICE_specifics_t asn_SPC_SignerIdentifier_specs_1;
extern asn_TYPE_member_t asn_MBR_SignerIdentifier_1[3];
extern asn_per_constraints_t asn_PER_type_SignerIdentifier_constr_1;

#ifdef __cplusplus
}
#endif

#endif	/* _SignerIdentifier_H_ */
#include <asn_internal.h>
